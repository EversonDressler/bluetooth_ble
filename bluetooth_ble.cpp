#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "MPU_6050_GATT.h"
#include "haw/MPU6050.h"
#include "btstack.h"
#include "pico/btstack_cyw43.h"

#define HEARTBEAT_PERIOD_MS 50
#define APP_AD_FLAGS 0x06
 
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint8_t adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    0x13, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'D', 'R', 'E', 'S', 'S', 'L', 'E', 'R', ' ', 'B', 'l', 'u', 'e', 'T', 'o', 'o', 't', 'h',
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
};

static const uint8_t adv_data_len = sizeof(adv_data);

// Variáveis globais
// bool le_notification_enabled[7]; // Corrigido para 6 posições (3 acel, 3 giro)
int le_notification_enabled;
hci_con_handle_t con_handle;
static float acel[3] = {0,0,0}, giro[3] = {0,0,0},  temperatura = 0;
uint16_t tx_temp = 0;
mpu6050_t mpu6050;

// Protótipos de Funções
void init_imu_mpu_6050();
void poll_imu_mpu_6050();
static void heartbeat_handler(struct btstack_timer_source *ts);
void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);
static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);

void init_imu_mpu_6050()
{
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    i2c_init(i2c_default, 400000);

    mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);

    if(!mpu6050_begin(&mpu6050))
    {
        while(true)
        {
            printf("Erro ao inicializar o sensor!\n");
            sleep_ms(500);
        }
    }

    mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);
    mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);
    mpu6050_set_temperature_measuring(&mpu6050, true);
    mpu6050_set_gyroscope_measuring(&mpu6050, true);
    mpu6050_set_accelerometer_measuring(&mpu6050, true);
    mpu6050_set_int_free_fall(&mpu6050, false);
    mpu6050_set_int_motion(&mpu6050, false);
    mpu6050_set_int_zero_motion(&mpu6050, false);
    mpu6050_set_motion_detection_threshold(&mpu6050, 2);
    mpu6050_set_motion_detection_duration(&mpu6050, 5);
    mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);
    mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
}

void poll_imu_mpu_6050()
{
    mpu6050_event(&mpu6050);
    mpu6050_vectorf_t *acel_temp = mpu6050_get_accelerometer(&mpu6050);
    mpu6050_vectorf_t *giro_temp = mpu6050_get_gyroscope(&mpu6050);
    temperatura = mpu6050_get_temperature_c(&mpu6050); // ou mpu6050_get_temperature_f()

    acel[0] = acel_temp->x;
    acel[1] = acel_temp->y;
    acel[2] = acel_temp->z;
    giro[0] = giro_temp->x;
    giro[1] = giro_temp->y;
    giro[2] = giro_temp->z;

    printf("acel x %.2f, acel y %.2f, acel z %.2f\n", acel[0], acel[1], acel[2]);
    printf("giro x %.2f, giro y %.2f, giro z %.2f\n", giro[0], giro[1], giro[2]);
    printf("temp %.2f \xC2\xB0""C\n", temperatura); // ou mpu6050_get_temperature_f()
}

static void heartbeat_handler(struct btstack_timer_source *ts)
{
    static uint32_t contador = 0;
    contador++;

    if (contador % 2 == 0)
    {
        poll_imu_mpu_6050();
        // for (int i = 0; i < 7; i++) {
            // if (le_notification_enabled[i]) {
            if (le_notification_enabled)
            {
                att_server_request_can_send_now_event(con_handle);
            }
        // }
    }

    static bool led = true;
    led = !led;

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    if(packet_type != HCI_EVENT_PACKET) return;

    uint8_t tipo_evento = hci_event_packet_get_type(packet);

    tx_temp = (int16_t)(temperatura * 100);

    switch (tipo_evento)
    {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            bd_addr_t end_local;
            gap_local_bd_addr(end_local);
            printf("Pilha Bluetooth esta rodando no endereco: %s", bd_addr_to_str(end_local));
        break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            for(int i = 0; i < 7; i++)
            {
                // le_notification_enabled[i] = false;
                le_notification_enabled = false;
            }
        break;

        case ATT_EVENT_CAN_SEND_NOW:
            // if (le_notification_enabled[0]){
                att_server_notify(con_handle, ATT_CHARACTERISTIC_e7890e92_ed43_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &acel[0], sizeof(acel[0]));
            // }
            // if (le_notification_enabled[1]){
                att_server_notify(con_handle, ATT_CHARACTERISTIC_9b788ed0_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &acel[1], sizeof(acel[1]));
            // }
            // if (le_notification_enabled[2]){
                att_server_notify(con_handle, ATT_CHARACTERISTIC_9b789218_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &acel[2], sizeof(acel[2]));
            // }
            // if (le_notification_enabled[3]){
                att_server_notify(con_handle, ATT_CHARACTERISTIC_9b789470_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &giro[0], sizeof(giro[0]));
            // }
            // if (le_notification_enabled[4]){
                att_server_notify(con_handle, ATT_CHARACTERISTIC_9b789632_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &giro[1], sizeof(giro[1]));
            // }
            // if (le_notification_enabled[5]){
                att_server_notify(con_handle, ATT_CHARACTERISTIC_9b7897d6_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &giro[2], sizeof(giro[2]));
            // }
            // if (le_notification_enabled[6]){
            
                att_server_notify(con_handle, ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE, (uint8_t *) &tx_temp, sizeof(tx_temp));
            // }
        break;

        default:
        break;
    }
}

static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size)
{
    switch (att_handle)
    {
        case ATT_CHARACTERISTIC_e7890e92_ed43_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *) &acel[0], sizeof(acel[0]),offset, buffer, buffer_size);    
        case ATT_CHARACTERISTIC_9b788ed0_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *) &acel[1], sizeof(acel[1]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_9b789218_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *) &acel[2], sizeof(acel[2]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_9b789470_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *) &giro[0], sizeof(giro[0]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_9b789632_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *) &giro[1], sizeof(giro[1]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_9b7897d6_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *) &giro[2], sizeof(giro[2]), offset, buffer, buffer_size);
        case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE:
            return att_read_callback_handle_blob((const uint8_t *) &temperatura, sizeof(temperatura), offset, buffer, buffer_size);
        default:
        break;
    }

    return 0;
}

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) 
{
    le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;

    if(le_notification_enabled)
    {
        con_handle = connection_handle;
        att_server_request_can_send_now_event(con_handle);
    }

    return 0;

    // switch (att_handle) {
    //     case ATT_CHARACTERISTIC_e7890e92_ed43_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
    //         le_notification_enabled[0] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    //     break;
    //     case ATT_CHARACTERISTIC_9b788ed0_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
    //         le_notification_enabled[1] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    //     break;
    //     case ATT_CHARACTERISTIC_9b789218_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
    //         le_notification_enabled[2] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    //     break;
    //     case ATT_CHARACTERISTIC_9b789470_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
    //         le_notification_enabled[3] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    //     break;
    //     case ATT_CHARACTERISTIC_9b789632_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
    //         le_notification_enabled[4] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    //     break;
    //     case ATT_CHARACTERISTIC_9b7897d6_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
    //         le_notification_enabled[5] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    //     break;
    //     case ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE:
    //         le_notification_enabled[6] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    //     break;
    // }
    // for(int i = 0; i < 7; i++) {
    //     if(le_notification_enabled[i]) {
    //         con_handle = connection_handle;
    //         att_server_request_can_send_now_event(con_handle);
    //     }
    // }
    // return 0;
}

int main()
{
    stdio_init_all();

    printf("Iniciando\n");

    if (cyw43_arch_init())
    {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    l2cap_init();
    sm_init();
    init_imu_mpu_6050();

    hci_event_callback_registration.callback = &packet_handler;

    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(packet_handler);

    heartbeat.process = &heartbeat_handler;

    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);
    att_server_init(profile_data, att_read_callback, att_write_callback);

    bd_addr_t endereco;

    memset(endereco, 0, sizeof(bd_addr_t));
    gap_advertisements_set_params(800, 800, 0, 0,endereco, 0x07, 0);
    gap_advertisements_set_data(adv_data_len, (uint8_t *) adv_data);
    gap_advertisements_enable(true);
    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
}










































// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
// #include "MPU_6050_GATT.h"
// #include "haw/MPU6050.h"
// #include "btstack.h"
// #include "pico/btstack_cyw43.h"

// /*
//  * Intervalo de tempo em Milisegundos utilizado para controlar a 'batida do coração'
//  * da aplicação, estratégia comum adotada por aplicações que utilizam BLE
//  */
// #define HEARTBEAT_PERIOD_MS 50

// /*
//  * Configura a habilitação das Flags de 'advertisement' (aviso) dos parâmetros do BLE
//  * o valor 0x06 corresponde à habilitação do modo BLE apenas nas frequências homologadas
//  */
// #define APP_AD_FLAGS 0x06

// /*
//  * estrutura de dados, utilizada para controlar o processo de 'heartbet' (batida do coração) 
//  * do BLE
//  */
// static btstack_timer_source_t heartbeat;

// /*
//  * Estrutura de dados utilizada para gerenciar as 'callbacks' (eventos) do BLE.
//  * a variável é nomeada em referência ao tipo de conexão que desempenha, onde 
//  * 'hci' significa Humam Control Interface, o que sinaliza o tipo de uso da aplicação
//  */
// static btstack_packet_callback_registration_t hci_event_callback_registration;


// /*
//  * dados de 'advertisement', utilizados na descoberta do módulo BLE por outros
//  * dispositivos/aplicações.
//  * 
//  * Cada linha é separada da Seguinte Maneira
//  * <TAMANHO TOTAL DOS DADOS DA LINHA>, <CAMPO_PARA_CONFIGURAR>, <VALOR DA CONFIGURAÇÃO>
//  * 
//  * onde cada <CAMPO_PARA_CONFIGURAÇÃO> vale 1 byte
//  */
// static uint8_t adv_data[] = {
//     // Flags gerais para descobrimento
//     0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
//     // Nome do dispositivo
//     0x0C, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', '2', 'W', ' ', 'I', 'M', 'U',
//     // Classe de Serviço padrão
//     0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
// };

// /*
//  * Tamanho total dos dados de 'advertisement'
//  */
// static const uint8_t adv_data_len = sizeof(adv_data);

// //Variáveis globais

// /*
//  * Vetor booleano utilizado para controlar quais atributos devem ser enviados quando alterados.
//  */
// bool le_notification_enabled[7];

// /*
//  * variável utilizada para armazenar o 'conection handler' (manipulador de conexões), utilizado
//  * em diversas partes do código, seu tipo é um 'hci_con_handle_t' (manipulador de conexões de controle de interface humana)
//  */
// hci_con_handle_t con_handle;

// /*
//  * variáveis estáticas utilizadas para armazenar os dados lidos do sensor
//  */
// static float acel[3] = {0,0,0}, giro[3] = {0,0,0},  temperatura = 0;

// /*
//  * variável que será utilizada na leitura da temperatura do MPU6050
//  */
// uint16_t tx_temp = 0;

// /*
//  * Estrutura de dados utilizada para armazenar os dados referentes ao Sensor MPU6050
//  */
// mpu6050_t mpu6050;

// //Protótipos de Funções

// /* 
//  * Função dedicada a inicializar o módulo MPU6050 no barramento I2C
//  */
// void init_imu_mpu_6050();
// /*
//  * Função de 'polling' dos dados do MPU6050 responsável por realizar as leituras do sensor
//  */
// void poll_imu_mpu_6050();

// /*
//  * Callback (evento/interrupção) 'packet_handler' utilizada para controlar as 'batidas do coração' da 
//  * Raspberry pi Pico no modo de operação com BLE
//  * 
//  * Parâmetros:
//  * struct btstack_timer_source *ts: Fonte de timer utilizada para controlar a operação do Bluetooth Low Energy
//  */
// static void heartbeat_handler(struct btstack_timer_source *ts) ;

// /*
//  * Callback (evento/interrupção) 'packet_handler' utilizada para controlar os pacotes recebidos/enviados pelo 
//  * módulo BLE.
//  * 
//  * Parâmetros:
//  * uint8_t packet_type: Parâmetro responsável por sinalizar o tipo do pacote, 
//  * uint16_t channel: Parâmetro responsável por sinalizar qual o canal do módulo BLE está em uso, 
//  * uint8_t *packet: Ponteiro contendo os dados do pacote, 
//  * uint16_t size: Parâmetro contento o tamanho do pacote recebido
//  */
// void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

// /*
//  * Callblack (evento/interrupção) 'att_read_callback' utilizada no momento em que uma leitura é solicitada 
//  * para o módulo BLE:
//  * 
//  * Parâmetros:
//  * hci_con_handle_t: connection_handle, Parâmetro utilizado para armazenar qual a conexão está solicitando leitura, muito 
//  * utilizado em situações com mais de uma conexão simultânea.
//  * 
//  * uint16_t att_handle: Parâmetro utilizado para armazenar qual atributo teve a leitura solicitada
//  * 
//  * uint16_t offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//  * se deseja ler
//  * 
//  * uint8_t * buffer: Parâmetro utilizado para onde deve se copiar os dados que estão sendo lidos
//  * 
//  * uint16_t buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados.
//  */
// static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size);

// /*
//  * Callblack (evento/interrupção) 'att_write_callback' utilizada no momento em que uma escrita é solicitada 
//  * para o módulo BLE:
//  * 
//  * Parâmetros:
//  * hci_con_handle_t: connection_handle, Parâmetro utilizado para armazenar qual a conexão está solicitando escrita, muito 
//  * utilizado em situações com mais de uma conexão simultânea.
//  * 
//  * uint16_t transaction_mode: Parâmetro utilizado para identificar o tipo de transação:
//  * Valores possíveis:
//  *   ATT_TRANSACTION_MODE_NONE (0): Escrita normal
//  *   ATT_TRANSACTION_MODE_ACTIVE (1): Parte de uma escrita longa (prepare write)
//  *   ATT_TRANSACTION_MODE_EXECUTE (2): Executar/descartar escrita longa acumulada
//  * os modos 1 e 2, se referem á grandes quantidades de dados sendo transportadas
//  * 
//  * uint16_t att_handle: Parâmetro utilizado para armazenar qual atributo teve a escrita solicitada
//  * 
//  * uint16_t offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//  * se deseja escrever
//  * 
//  * uint8_t * buffer: Parâmetro utilizado de onde deve se copiar os dados que estão sendo escritos
//  * 
//  * uint16_t buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados.
//  */
// static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);


// /* 
//  * Função dedicada a inicializar o módulo MPU6050 no barramento I2C
//  */
// void init_imu_mpu_6050() {
//     /*
//      * Inicializa os Pinos padrão do I2C
//      */
//     gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
//     gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    
//     /*
//      * Configura os Pinos padrão do I2C para funcionarem efetivamente
//      * como um periférico I2C
//      */
//     gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
//     gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

//     /*
//      * Configura os Resistores internos de Pull-up para os pinos do 
//      * periférico I2C
//      */
//     gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
//     gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

//     /*
//      * Inicializa o periférico I2C padrão, com uma frequência de funcionamento
//      * de 400Khz (40000 Hz)
//      */
//     i2c_init(i2c_default, 400000);

//     /*
//      * Inicializa o Sensor MPU6050 no barramento I2C padrão com o endereço
//      * Padrão do módulo
//      */
//     mpu6050 = mpu6050_init(i2c_default, MPU6050_ADDRESS_A0_GND);

//     /*
//      * Inicializa a comunicação do módulo MPU6050
//      */
//     if(!mpu6050_begin(&mpu6050)) {
//         while(true){
//             printf("Erro ao inicializar o sensor!\n");
//             sleep_ms(500);
//         }
//     }

//     /*
//      * configura as escalas do giroscópio
//      */
//     mpu6050_set_scale(&mpu6050, MPU6050_SCALE_2000DPS);

//     /*
//      * Configura as escalas do acelerômetro
//      */
//     mpu6050_set_range(&mpu6050, MPU6050_RANGE_16G);

//     /*
//      * Habilita o acelerômetro o giroscópio e o termômetro interno do 
//      * MPU6050
//      */
//     mpu6050_set_temperature_measuring(&mpu6050, true);
//     mpu6050_set_gyroscope_measuring(&mpu6050, true);
//     mpu6050_set_accelerometer_measuring(&mpu6050, true);

//     /*
//      * Desabilita todas as interrupções do módulo MPU6050
//      */
//     mpu6050_set_int_free_fall(&mpu6050, false);
//     mpu6050_set_int_motion(&mpu6050, false);
//     mpu6050_set_int_zero_motion(&mpu6050, false);

//     /*
//      * Define o limiar para a detecção de aceleração
//      */
//     mpu6050_set_motion_detection_threshold(&mpu6050, 2);
    
//     /*
//      * Combinado ao parâmetro anterior, determina quanto tempo o movimento 
//      * deve durar para ser detectado
//      */
//     mpu6050_set_motion_detection_duration(&mpu6050, 5);

//     /*
//      * Define o limiar para a detecção do sensor parado
//      */
//     mpu6050_set_zero_motion_detection_threshold(&mpu6050, 4);

//     /*
//      * Combinado ao parâmetro anterior, determina quanto tempo a paralisação 
//      * deve durar para ser detectado
//      */
//     mpu6050_set_zero_motion_detection_duration(&mpu6050, 2);
// }


// /*
//  * Função de 'polling' dos dados do MPU6050 responsável por realizar as leituras do sensor
//  */
// void poll_imu_mpu_6050(){
//     //Realiza a leitura do sensor
//     mpu6050_event(&mpu6050);

//     //Copia os dados lidos para uma variável local temporária
//     mpu6050_vectorf_t *acel_temp = mpu6050_get_accelerometer(&mpu6050);
//     mpu6050_vectorf_t *giro_temp = mpu6050_get_gyroscope(&mpu6050);

//     //Salva os valores nas variáveis globais
//     acel[0] = acel_temp->x;
//     acel[1] = acel_temp->y;
//     acel[2] = acel_temp->z;

//     giro[0] = giro_temp->x;
//     giro[1] = giro_temp->y;
//     giro[2] = giro_temp->z;

//     //printa as informações no console, utilizado para debug
//     printf("acel x %.2f, acel y %.2f, acel z %.2f\n", acel[0], acel[1], acel[2]);
//     printf("giro x %.2f, giro y %.2f, giro z %.2f\n", giro[0], giro[1], giro[2]);
// }

// /*
//  * Callback (evento/interrupção) 'packet_handler' utilizada para controlar as 'batidas do coração' da 
//  * Raspberry pi Pico no modo de operação com BLE
//  * 
//  * Parâmetros:
//  * struct btstack_timer_source *ts: Fonte de timer utilizada para controlar a operação do Bluetooth Low Energy
//  */
// static void heartbeat_handler(struct btstack_timer_source *ts)
// {
//     static uint32_t contador = 0;
//     contador++;

    
//     /*
//      * Caso o contador seja um númerp par, realiza a leitura dos dados do sensor
//      * e os envia caso necessário
//      */
//     if (contador % 2 == 0) {
//         poll_imu_mpu_6050();
//         for (int i = 0; i < 7; i++) {
//             if (le_notification_enabled) {
//                 att_server_request_can_send_now_event(con_handle);
//             }
//         }
//     }

//     //Faz o led piscar
//     static bool led  = true;
//     led = !led;
//     cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led);

//     /*
//      *
//      * Reconfigura o Timer para que ele seja disparado novamente de acordo com o tempo 
//      * definido na MACRO HEARTBEAT_PERIOD_MS
//      */
//     btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
//     btstack_run_loop_add_timer(ts);
// }

// /*
//  * Callback (evento/interrupção) 'packet_handler' utilizada para controlar os pacotes recebidos/enviados pelo 
//  * módulo BLE.
//  * 
//  * Parâmetros:
//  * uint8_t packet_type: Parâmetro responsável por sinalizar o tipo do pacote, 
//  * uint16_t channel: Parâmetro responsável por sinalizar qual o canal do módulo BLE está em uso, 
//  * uint8_t *packet: Ponteiro contendo os dados do pacote, 
//  * uint16_t size: Parâmetro contento o tamanho do pacote recebido
//  */
// void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
//     // Caso o pacote recebido não seja do tipo HCI_EVENT_PACKET, ele é descartado
//     if(packet_type != HCI_EVENT_PACKET) return;

//     //recuperando o tipo dos dados do pacote
//     uint8_t tipo_evento = hci_event_packet_get_type(packet);

//     switch (tipo_evento)
//     {
//         //Caso seja o estado de evento
//         case BTSTACK_EVENT_STATE:
//             // Recuperamos se é um evento de conectado/copnfigurado corretamente, do contrário descartamos
//             if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
            
//             //Recuperamos o endereço local do módulo bluetooth
//             bd_addr_t end_local;
//             gap_local_bd_addr(end_local);
//             printf("Pilha Bluetooth esta rodando no endereco: %s", bd_addr_to_str(end_local));
//         break;

//         /*
//          * Caso uma desconexão seja detectada, todos os dados tem suas notificações canceladas
//          */
//         case HCI_EVENT_DISCONNECTION_COMPLETE:
//             for( int i = 0; i < 7; i++) {
//                 le_notification_enabled[i] = false;
//             }
//         break;

//         /*
//          * Caso seja uma solicitação de notificação, verificamos quais notificações estão habilitadas, e enviamos os dados correspondentes
//          */
//         case ATT_EVENT_CAN_SEND_NOW:
//             if (le_notification_enabled[0]){
//                 /*
//                  * Função att_server_notify, utilizada para notificar clientes, conectados ao servidor (Raspberry Pi Pico)
//                  *                    
//                  * Parâmetros:
//                  * 
//                  * <1> (con_handle), indica qual conexão deve ser utilizada no envio dos dados, útil para múltiplas conexões
//                  * <2> ATT_CHARACTERISTIC...01_VALUE_HANDLE, indica qual característica será notificada, lembrar sempre de utilizar o 'VALUE_HANDLE'
//                  * para enviar valores.
//                  * <3> (uint8_t *) <dados>, indica os dados que devem ser transmitidos, devem sempre receber um 'cast' para um ponteiro de 8 bits não sinalizados
//                  * (uint8_t *) para um correto transporte
//                  * <4> sizeof(<dados>), indica o tamanho total em bytes do valor que se deseja transmitir
//                  */
//                 att_server_notify(con_handle, ATT_CHARACTERISTIC_e7890e92_ed43_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &acel[0], sizeof(acel[0]));
//             }
            
//             if (le_notification_enabled[1]){
//                 /*
//                  * Função att_server_notify, utilizada para notificar clientes, conectados ao servidor (Raspberry Pi Pico)
//                  *                    
//                  * Parâmetros:
//                  * 
//                  * <1> (con_handle), indica qual conexão deve ser utilizada no envio dos dados, útil para múltiplas conexões
//                  * <2> ATT_CHARACTERISTIC...01_VALUE_HANDLE, indica qual característica será notificada, lembrar sempre de utilizar o 'VALUE_HANDLE'
//                  * para enviar valores.
//                  * <3> (uint8_t *) <dados>, indica os dados que devem ser transmitidos, devem sempre receber um 'cast' para um ponteiro de 8 bits não sinalizados
//                  * (uint8_t *) para um correto transporte
//                  * <4> sizeof(<dados>), indica o tamanho total em bytes do valor que se deseja transmitir
//                  */
//                 att_server_notify(con_handle, ATT_CHARACTERISTIC_9b788ed0_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &acel[1], sizeof(acel[1]));
//             }

//             if (le_notification_enabled[2]){
//                 /*
//                  * Função att_server_notify, utilizada para notificar clientes, conectados ao servidor (Raspberry Pi Pico)
//                  *                    
//                  * Parâmetros:
//                  * 
//                  * <1> (con_handle), indica qual conexão deve ser utilizada no envio dos dados, útil para múltiplas conexões
//                  * <2> ATT_CHARACTERISTIC...01_VALUE_HANDLE, indica qual característica será notificada, lembrar sempre de utilizar o 'VALUE_HANDLE'
//                  * para enviar valores.
//                  * <3> (uint8_t *) <dados>, indica os dados que devem ser transmitidos, devem sempre receber um 'cast' para um ponteiro de 8 bits não sinalizados
//                  * (uint8_t *) para um correto transporte
//                  * <4> sizeof(<dados>), indica o tamanho total em bytes do valor que se deseja transmitir
//                  */
//                 att_server_notify(con_handle, ATT_CHARACTERISTIC_9b789218_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &acel[2], sizeof(acel[2]));
//             }

//             if (le_notification_enabled[3]){
//                 /*
//                  * Função att_server_notify, utilizada para notificar clientes, conectados ao servidor (Raspberry Pi Pico)
//                  *                    
//                  * Parâmetros:
//                  * 
//                  * <1> (con_handle), indica qual conexão deve ser utilizada no envio dos dados, útil para múltiplas conexões
//                  * <2> ATT_CHARACTERISTIC...01_VALUE_HANDLE, indica qual característica será notificada, lembrar sempre de utilizar o 'VALUE_HANDLE'
//                  * para enviar valores.
//                  * <3> (uint8_t *) <dados>, indica os dados que devem ser transmitidos, devem sempre receber um 'cast' para um ponteiro de 8 bits não sinalizados
//                  * (uint8_t *) para um correto transporte
//                  * <4> sizeof(<dados>), indica o tamanho total em bytes do valor que se deseja transmitir
//                  */
//                 att_server_notify(con_handle, ATT_CHARACTERISTIC_9b789470_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &giro[0], sizeof(giro[0]));
//             }

//             if (le_notification_enabled[4]){
//                 /*
//                  * Função att_server_notify, utilizada para notificar clientes, conectados ao servidor (Raspberry Pi Pico)
//                  *                    
//                  * Parâmetros:
//                  * 
//                  * <1> (con_handle), indica qual conexão deve ser utilizada no envio dos dados, útil para múltiplas conexões
//                  * <2> ATT_CHARACTERISTIC...01_VALUE_HANDLE, indica qual característica será notificada, lembrar sempre de utilizar o 'VALUE_HANDLE'
//                  * para enviar valores.
//                  * <3> (uint8_t *) <dados>, indica os dados que devem ser transmitidos, devem sempre receber um 'cast' para um ponteiro de 8 bits não sinalizados
//                  * (uint8_t *) para um correto transporte
//                  * <4> sizeof(<dados>), indica o tamanho total em bytes do valor que se deseja transmitir
//                  */
//                 att_server_notify(con_handle, ATT_CHARACTERISTIC_9b789632_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &giro[1], sizeof(giro[1]));
//             }

//             if (le_notification_enabled[5]){
//                 /*
//                  * Função att_server_notify, utilizada para notificar clientes, conectados ao servidor (Raspberry Pi Pico)
//                  *                    
//                  * Parâmetros:
//                  * 
//                  * <1> (con_handle), indica qual conexão deve ser utilizada no envio dos dados, útil para múltiplas conexões
//                  * <2> ATT_CHARACTERISTIC...01_VALUE_HANDLE, indica qual característica será notificada, lembrar sempre de utilizar o 'VALUE_HANDLE'
//                  * para enviar valores.
//                  * <3> (uint8_t *) <dados>, indica os dados que devem ser transmitidos, devem sempre receber um 'cast' para um ponteiro de 8 bits não sinalizados
//                  * (uint8_t *) para um correto transporte
//                  * <4> sizeof(<dados>), indica o tamanho total em bytes do valor que se deseja transmitir
//                  */
//                 att_server_notify(con_handle, ATT_CHARACTERISTIC_9b7897d6_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE, (uint8_t *) &giro[2], sizeof(giro[2]));
//             }
//         default:
//             break;
//     }
   
// }

// /*
//  * Callblack (evento/interrupção) 'att_read_callback' utilizada no momento em que uma leitura é solicitada 
//  * para o módulo BLE:
//  * 
//  * Parâmetros:
//  * hci_con_handle_t: connection_handle, Parâmetro utilizado para armazenar qual a conexão está solicitando leitura, muito 
//  * utilizado em situações com mais de uma conexão simultânea.
//  * 
//  * uint16_t att_handle: Parâmetro utilizado para armazenar qual atributo teve a leitura solicitada
//  * 
//  * uint16_t offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//  * se deseja ler
//  * 
//  * uint8_t * buffer: Parâmetro utilizado para onde deve se copiar os dados que estão sendo lidos
//  * 
//  * uint16_t buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados.
//  */
// static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size) {
//     //Verifica qual atributo precisa ser lido
//     switch (att_handle)
//     {
//         case ATT_CHARACTERISTIC_e7890e92_ed43_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
//             /*
//              * Função 'att_read_callback_handle_blob', utilizada para copiar os dados lidos para o buffer de leitura
//              * Parâmetros:
//              * <1> (const uint8_t *) &acel[0], ponteiro contendo os dados que devem ser enviados
//              * <2> sizeof(acel[0]), tamanho total em bytes do valor a ser enviado
//              * <3>  offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//              * se deseja ler, deve-se utilizar o mesmo parâmetro recebido na função
//              * <4> buffer: Parâmetro utilizado para onde deve se copiar os dados que estão sendo lidos, deve-se utilizar o mesmo 
//              * parâmetro recebido na função.
//              * <5> buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados, deve-se utilizar o mesmo parâmetro 
//              * recebido na função.
//              */
//             return att_read_callback_handle_blob((const uint8_t *) &acel[0], sizeof(acel[0]),offset, buffer, buffer_size);    
//         break;
    
//         case ATT_CHARACTERISTIC_9b788ed0_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
//             /*
//              * Função 'att_read_callback_handle_blob', utilizada para copiar os dados lidos para o buffer de leitura
//              * Parâmetros:
//              * <1> (const uint8_t *) &acel[0], ponteiro contendo os dados que devem ser enviados
//              * <2> sizeof(acel[0]), tamanho total em bytes do valor a ser enviado
//              * <3>  offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//              * se deseja ler, deve-se utilizar o mesmo parâmetro recebido na função
//              * <4> buffer: Parâmetro utilizado para onde deve se copiar os dados que estão sendo lidos, deve-se utilizar o mesmo 
//              * parâmetro recebido na função.
//              * <5> buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados, deve-se utilizar o mesmo parâmetro 
//              * recebido na função.
//              */
//             return att_read_callback_handle_blob((const uint8_t *) &acel[1], sizeof(acel[1]), offset, buffer, buffer_size);
//         break;

//         case ATT_CHARACTERISTIC_9b789218_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
//             /*
//              * Função 'att_read_callback_handle_blob', utilizada para copiar os dados lidos para o buffer de leitura
//              * Parâmetros:
//              * <1> (const uint8_t *) &acel[0], ponteiro contendo os dados que devem ser enviados
//              * <2> sizeof(acel[0]), tamanho total em bytes do valor a ser enviado
//              * <3>  offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//              * se deseja ler, deve-se utilizar o mesmo parâmetro recebido na função
//              * <4> buffer: Parâmetro utilizado para onde deve se copiar os dados que estão sendo lidos, deve-se utilizar o mesmo 
//              * parâmetro recebido na função.
//              * <5> buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados, deve-se utilizar o mesmo parâmetro 
//              * recebido na função.
//              */
//             return att_read_callback_handle_blob((const uint8_t *) &acel[2], sizeof(acel[2]), offset, buffer, buffer_size);
//         break;

//         case ATT_CHARACTERISTIC_9b789470_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
//             /*
//              * Função 'att_read_callback_handle_blob', utilizada para copiar os dados lidos para o buffer de leitura
//              * Parâmetros:
//              * <1> (const uint8_t *) &acel[0], ponteiro contendo os dados que devem ser enviados
//              * <2> sizeof(acel[0]), tamanho total em bytes do valor a ser enviado
//              * <3>  offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//              * se deseja ler, deve-se utilizar o mesmo parâmetro recebido na função
//              * <4> buffer: Parâmetro utilizado para onde deve se copiar os dados que estão sendo lidos, deve-se utilizar o mesmo 
//              * parâmetro recebido na função.
//              * <5> buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados, deve-se utilizar o mesmo parâmetro 
//              * recebido na função.
//              */
//             return att_read_callback_handle_blob((const uint8_t *) &giro[0], sizeof(giro[0]), offset, buffer, buffer_size);
//         break;

//         case ATT_CHARACTERISTIC_9b789632_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
//             /*
//              * Função 'att_read_callback_handle_blob', utilizada para copiar os dados lidos para o buffer de leitura
//              * Parâmetros:
//              * <1> (const uint8_t *) &acel[0], ponteiro contendo os dados que devem ser enviados
//              * <2> sizeof(acel[0]), tamanho total em bytes do valor a ser enviado
//              * <3>  offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//              * se deseja ler, deve-se utilizar o mesmo parâmetro recebido na função
//              * <4> buffer: Parâmetro utilizado para onde deve se copiar os dados que estão sendo lidos, deve-se utilizar o mesmo 
//              * parâmetro recebido na função.
//              * <5> buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados, deve-se utilizar o mesmo parâmetro 
//              * recebido na função.
//              */
//             return att_read_callback_handle_blob((const uint8_t *) &giro[1], sizeof(giro[1]), offset, buffer, buffer_size);
//         break;

//         case ATT_CHARACTERISTIC_9b7897d6_f04d_11ed_a05b_0242ac120003_01_VALUE_HANDLE:
//             /*
//              * Função 'att_read_callback_handle_blob', utilizada para copiar os dados lidos para o buffer de leitura
//              * Parâmetros:
//              * <1> (const uint8_t *) &acel[0], ponteiro contendo os dados que devem ser enviados
//              * <2> sizeof(acel[0]), tamanho total em bytes do valor a ser enviado
//              * <3>  offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//              * se deseja ler, deve-se utilizar o mesmo parâmetro recebido na função
//              * <4> buffer: Parâmetro utilizado para onde deve se copiar os dados que estão sendo lidos, deve-se utilizar o mesmo 
//              * parâmetro recebido na função.
//              * <5> buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados, deve-se utilizar o mesmo parâmetro 
//              * recebido na função.
//              */
//             return att_read_callback_handle_blob((const uint8_t *) &giro[2], sizeof(giro[2]), offset, buffer, buffer_size);
//         break;

//     default:
//         break;
//     }

//     return 0;
// }

// /*
//  * Callblack (evento/interrupção) 'att_write_callback' utilizada no momento em que uma escrita é solicitada 
//  * para o módulo BLE:
//  * 
//  * Parâmetros:
//  * hci_con_handle_t: connection_handle, Parâmetro utilizado para armazenar qual a conexão está solicitando escrita, muito 
//  * utilizado em situações com mais de uma conexão simultânea.
//  * 
//  * uint16_t transaction_mode: Parâmetro utilizado para identificar o tipo de transação:
//  * Valores possíveis:
//  *   ATT_TRANSACTION_MODE_NONE (0): Escrita normal
//  *   ATT_TRANSACTION_MODE_ACTIVE (1): Parte de uma escrita longa (prepare write)
//  *   ATT_TRANSACTION_MODE_EXECUTE (2): Executar/descartar escrita longa acumulada
//  * os modos 1 e 2, se referem á grandes quantidades de dados sendo transportadas
//  * 
//  * uint16_t att_handle: Parâmetro utilizado para armazenar qual atributo teve a escrita solicitada
//  * 
//  * uint16_t offset: Parâmetro utilizado para indicar um deslocamento (offset) dentro do buffer de dados do atributos que 
//  * se deseja escrever
//  * 
//  * uint8_t * buffer: Parâmetro utilizado de onde deve se copiar os dados que estão sendo escritos
//  * 
//  * uint16_t buffer_size Parâmetro utilizado para informar o tamanho dos dados copiados.
//  */
// static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
//     //desomentar para habilitar a notificação para todos os atributos
//     //le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;

//     //Caso habilite a notificação geral, comente este bloco de códdigo
//     switch (att_handle) {
//         /*
//          * Como todos os parâmetros são apenas leitura o único dado que pode ser recebido é o pedido de notificação 
//          */
//         case ATT_CHARACTERISTIC_e7890e92_ed43_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
//             /*
//              * a Função 'little_endian_read_16' faz a leitura de um valor de 16 bits a partir de uma posição 
//              * específica, comparando o resultado da leitura com o valor 'GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION'
//              * e armazena o resultado em 'le_notification_enabled', de acordo com a posição necessária
//              */
//             le_notification_enabled[0] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
//         break;

//         case ATT_CHARACTERISTIC_9b788ed0_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
//             /*
//              * a Função 'little_endian_read_16' faz a leitura de um valor de 16 bits a partir de uma posição 
//              * específica, comparando o resultado da leitura com o valor 'GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION'
//              * e armazena o resultado em 'le_notification_enabled', de acordo com a posição necessária
//              */
//             le_notification_enabled[1] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
//         break;

//         case ATT_CHARACTERISTIC_9b789218_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
//             /*
//              * a Função 'little_endian_read_16' faz a leitura de um valor de 16 bits a partir de uma posição 
//              * específica, comparando o resultado da leitura com o valor 'GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION'
//              * e armazena o resultado em 'le_notification_enabled', de acordo com a posição necessária
//              */
//             le_notification_enabled[2] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
//         break;

//         case ATT_CHARACTERISTIC_9b789470_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
//             /*
//              * a Função 'little_endian_read_16' faz a leitura de um valor de 16 bits a partir de uma posição 
//              * específica, comparando o resultado da leitura com o valor 'GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION'
//              * e armazena o resultado em 'le_notification_enabled', de acordo com a posição necessária
//              */
//             le_notification_enabled[3] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
//         break;

//         case ATT_CHARACTERISTIC_9b789632_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
//             /*
//              * a Função 'little_endian_read_16' faz a leitura de um valor de 16 bits a partir de uma posição 
//              * específica, comparando o resultado da leitura com o valor 'GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION'
//              * e armazena o resultado em 'le_notification_enabled', de acordo com a posição necessária
//              */
//             le_notification_enabled[4] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
//         break;

//         case ATT_CHARACTERISTIC_9b7897d6_f04d_11ed_a05b_0242ac120003_01_CLIENT_CONFIGURATION_HANDLE:
//             /*
//              * a Função 'little_endian_read_16' faz a leitura de um valor de 16 bits a partir de uma posição 
//              * específica, comparando o resultado da leitura com o valor 'GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION'
//              * e armazena o resultado em 'le_notification_enabled', de acordo com a posição necessária
//              */
//             le_notification_enabled[5] = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
//         break;
//     }


//     // Verifica cada uma das posições e envia os dados para os clientes conectados
//     for(int i = 0; i < 7; i++) {
//         if(le_notification_enabled[i]) {
//             con_handle = connection_handle;
//             att_server_request_can_send_now_event(con_handle);
//         }
//     }
//     return 0;
// }

// int main()
// {
//     //inicia as bibliotecas de I/O do microcontrolador
//     stdio_init_all();

//     printf("Iniciando\n");

//     // Initialise the Wi-Fi chip
//     if (cyw43_arch_init()) {
//         printf("Wi-Fi init failed\n");
//         return -1;
//     }

//     //Inicializa estruturas internas do bluetooth
//     l2cap_init();
//     sm_init();

//     //Inicializa o sensor MPU6050
//     init_imu_mpu_6050();

//     /*
//      * Configura a callback 'packet_handler' para os protocolos
//      * HCI
//      */
//     hci_event_callback_registration.callback = &packet_handler;
//     hci_add_event_handler(&hci_event_callback_registration);
  
//     /*
//      * registra o packet handler para o servidor de atributos
//      */
//     att_server_register_packet_handler(packet_handler);

//     /*
//      * Configura o heartbeat (batida de coração) da aplicação BLE
//      */
//     heartbeat.process = &heartbeat_handler;
//     btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
//     btstack_run_loop_add_timer(&heartbeat);
 
//     /*
//      * Inicializa o servidor de atributos do BLE
//      */
//     att_server_init(profile_data, att_read_callback, att_write_callback);

//     bd_addr_t endereco;
//     memset(endereco, 0, sizeof(bd_addr_t));

//     /*
//      * configura os parâmetros para o modo de 'generic atribute profile'
//      * 
//      * Parâmetros:
//      * <1> 800, intervalo mínimo para a atualização dos parâmetros
//      * <2> 800, intervalo máximo para a atualização dos parâmetros 
//      * <3> 'adv_type' (0), configura o tiop de 'advertisement':
//      *      0x00 (ADV_IND)	Advertising normal e conectável
//      *      0x01 (ADV_DIRECT_IND)	Direcionado e conectável
//      *      0x02 (ADV_SCAN_IND)	Escaneável, mas não conectável
//      *      0x03 (ADV_NONCONN_IND)	Não conectável, broadcast
//      * <4> 'tipo de endereço direto' (0):
//      *      0x00	Endereço público
//      *      0x01	Endereço aleatório
//      * <5> 'direct_address' (endereco) , caso o endereço não seja público:
//      *      Usado apenas se adv_type == ADV_DIRECT_IND.
//      * <6> 'mapa de canais' (0x07):
//      *      0	Canal 37
//      *      1	Canal 38
//      *      2	Canal 39
//      * <7> política de filtro (0)
//      *      0x00	Todos os dispositivos (padrão)
//      *      0x01	Apenas dispositivos da whitelist
//      *      0x02	Todos podem escanear e somente os dispositivos da whitelist podem conectar
//      *      0x03	Todos podem escanear e somente os dispositivos da whitelist podem conectar
//      * 
//      */
//     gap_advertisements_set_params(800, 800, 0, 0,endereco, 0x07, 0);

//     /*
//      * Configura os parâmetros para 'advertisement'
//      */
//     gap_advertisements_set_data(adv_data_len, (uint8_t *) adv_data);

//     /*
//      * habiilita o 'descobrimento' do dispositivo
//      */
//     gap_advertisements_enable(true);

//     /*
//      * Liga o módulo bluetooth
//      */
//     hci_power_control(HCI_POWER_ON);

//     //Loop infinito da btstack
//     btstack_run_loop_execute();
// }

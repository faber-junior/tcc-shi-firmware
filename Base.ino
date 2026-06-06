#include "HX711.h"             // Biblioteca para comunicação com o conversor A/D da balança
#include <Preferences.h>       // Biblioteca padrão para leitura/escrita na memória flash (NVS)
#include <WiFi.h>              // Biblioteca padrão para conexão WiFi
#include <WiFiManager.h>       // Biblioteca para gerenciamento de rede (Portal Cativo)
#include "time.h"              // Biblioteca padrão do C++ para lidar com relógio e data
#include <WebSocketsClient.h>  // Biblioteca para gerenciar clientes WebSocket (by Markus Sattler)
#include <ArduinoJson.h>       // Biblioteca para lidar com JSONs

/* --- Structs --- */
/*==================================================================================*/
typedef struct {
  uint8_t R; // Pino Vermelho
  uint8_t G; // Pino Verde
  uint8_t B; // Pino Azul
} LedRGB;

typedef struct {
  int frequency; // Frequência da nota em Hertz
  int duration;  // Duração da nota em milissegundos
} Note;

typedef struct {
  int hour;
  int minute;
} Hour;
/*==================================================================================*/

/* --- Pinos --- */
/*==================================================================================*/
const uint8_t HX711_DT = 34;        // Pino de dados do HX711
const uint8_t HX711_SCK = 2;        // Pino de clock do HX711
const uint8_t RESTORE_DEFAULTS = 0; // Botão físico "BOOT/EN" nativo do ESP32
const uint8_t BUZZER = 15;          // Pino de controle do buzzer passivo
const LedRGB LEDS[5] = {
  {27, 14, 32}, // LED 0
  {33, 25, 26}, // LED 1
  {23, 22, 21}, // LED 2
  {17, 16,  4}, // LED 3
  {13, 19, 18}  // LED 4
};
/*==================================================================================*/

/* ---  Configurações padrão --- */
/*==================================================================================*/
// Parâmetros de calibragem da balança (Valores de padrões calculados com um peso de 1870g)
const long SCALE_OFFSET = -14988;  // offset padrão
const float SCALE_DIVIDER = 422.6; // scacle_divider padrão

// Configurações de funcionamento para a máquina de estados
const Hour ACTIVE_START_HOUR = {7, 0}; // Horário de inicio das atividades da balança
const Hour ACTIVE_END_HOUR = {17, 0};// Horário de fim das atividades da balança
const unsigned long GRACE_PERIOD = 900000; // Periodo de carência (15 minutos em ms)

// Configurações de hidratação
const int DAILY_GOAL = 2000; // Meta diaria
/*==================================================================================*/

/* --- Constantes e parâmetros --- */
/*==================================================================================*/
// Configurações do Backend
const char* WS_HOST = "tcc-shi-api.onrender.com"; // Removido o https:// para evitar Erro 400
const int WS_PORT = 443; // Porta segura do Render
const char* TOKEN_ACESSO = "shi-balanca-001"; // Token fixo da balança

// Constantes dos filtros
const int NUM_SAMPLES_MEDIAN = 5;   // Amostras do Filtro de Mediana
const int NUM_SAMPLES_FILTERED = 5; // Amostras da Filtragem Final

// Configurações de WiFi e Access Point
const char *AP_NAME = "Balanca SHI";     // Nome da rede WiFi do Access Point
const char *AP_PASSWORD = "balanca_shi";  // Senha do WiFi do Access Point
const unsigned long WIFI_CHECK_INTERVAL = 10000; // Intervalo de checagem de conexão WiFi

// Tempo e Sincronização (NTP)
const char *NTP_SERVER = "pool.ntp.org"; // Servidor público mundial de tempo
const long GMT_OFFSET_SEC = -10800;      // Fuso horário do Brasil (UTC -3 horas convertido em segundos)
const int DAYLIGHT_OFFSET_SEC = 0;       // Ajuste de horário de verão (0 = desligado)

// Para o calculo do consumo
const int NOISE_TOLERANCE = 3; // Goles menores que 2mL são considerados ruído e ignorados (Porque podem estar proximo da borda entre um e outro ex: 56.9 - 57.1)
const unsigned long SETTLING_TIME = 1500; // Tempo de espera (ms) para a estabilização da leitura

// Animações (Melodias)
const Note RED_NOTES[] = { {784, 100}, {0, 80}, {1046, 100}, {0, 80}, {1175, 100}, {0, 80}, {1318, 100}, {0, 80}, {1568, 300} }; // Alerta Crítico
const Note YELLOW_NOTES[] = { {880, 150}, {0, 100}, {784, 250} }; // Aviso de Atenção
const Note GREEN_NOTES[] = { {523, 100}, {0, 50}, {659, 100}, {0, 50}, {784, 200} }; // Sucesso
/*==================================================================================*/

/* --- Variáveis globais e instâncias --- */
/*==================================================================================*/
HX711 scale; // Instância do HX711
WebSocketsClient webSocket; // Classe pura de WebSocket
String socketIO_url; 
unsigned long last_ws_ping = 0;
unsigned long last_realtime_send = 0;

// Vetores de armazenamento para os filtros
float samples_median[NUM_SAMPLES_MEDIAN] = {0.0};
float sample_ema = 0.0;
float weight = 0.0;    
float median_weight = 0.0;
float filtered_weight = 0.0;

// Variáveis de controle de estado da rede
unsigned long last_wifi_check = 0;
bool wifi_was_connected = false;

// Configurações de funcionamento
Hour active_start_hour = ACTIVE_START_HOUR; 
Hour active_end_hour = ACTIVE_END_HOUR; 
unsigned long grace_period = GRACE_PERIOD; 
int daily_goal = DAILY_GOAL; 
int container_weight = 0; // Mudar para zero depois de integrar ao web app

// Controle consumo
int last_recorded_weight = 0;        
bool is_container_present = false;       
bool is_waiting_stability = false;       
unsigned long time_container_placed = 0; 
int current_consumed_volume = 0;

// Dados de hidratação do usuario
int daily_consumed = 0;
time_t last_sip_time = 0; 
time_t last_alert_time = 0; 
int offline_volume = 0; // Volume consumido enquanto a balança estava offline

// Controle da máquina de estados
enum AlertState { IDLE, GREEN, YELLOW, RED }; 
AlertState current_state = IDLE; 
time_t now; // Horário
int current_ideal_volume = 0;
int deficit = 0;
bool is_active = false; // Está dentro da janela de tempo?

// Controle de tempo 
bool was_active = false; // Estava dentro da janela de tempo?
int last_reset_day = 0; // Ultimo dia em que a balança resetou

// Variaveis para o motor de animacoes
AlertState current_playing_alert = IDLE; 
bool is_alert_pausing = false; // O alerta está no intervalo entre as 3 repetições?
unsigned long alert_pause_start = 0; // Instante exato desde o inicio do ultimo ciclo da alerta
unsigned long current_animation_interval = 0; // Intervalo da animacao atual
unsigned long previous_animation_time = 0; // Instante exato do ultima animacao
unsigned long last_alert_instant = 0; // Instante exato do fim do ultimo alerta
int note_index = 0; // Indice atual da nota na melodia
int alert_repeat_count = 0; // Contagem de repetições da melodia atual

// Variaveis para depuração
bool is_test_alert = false; // Flag para ignorar atualização de tempo em testes pelo Serial
bool time_acceleration_active = false;
unsigned long simulated_time_offset = 0;
unsigned long last_accel_tick = 0;
unsigned long real_grace_period = 0;
unsigned int acceleration_factor = 60; // 60 segundos simulados a cada segundo real
time_t accelerated_time;
time_t simulated_last_sip_time = 0;
time_t simulated_last_alert_time = 0;

// Variáveis temporárias para calibração física
bool cancel_calibration_flag = false;
long temp_offset = 0;
float temp_scale_divider = 0.0;

// Declaração de funções para o compilador
void restoreDefaults();
void triggerAlert(AlertState type);
/*==================================================================================*/

/* --- Definição de configurações e parâmetros (Partição "config" da NVS) --- */
/*==================================================================================*/
void setActiveStartHour(Hour active_start)
{
  active_start_hour = active_start; // Define a variável global

  // Atualiza valor na NVS
  Preferences preferences;
  preferences.begin("config", false);
  preferences.putInt("start_hour", active_start_hour.hour);
  preferences.putInt("start_minute", active_start_hour.minute);
  preferences.end(); 
}

void setActiveEndHour(Hour active_end)
{
  active_end_hour = active_end; // Define a variável global

  // Atualiza valor na NVS
  Preferences preferences;
  preferences.begin("config", false); 
  preferences.putInt("end_hour", active_end_hour.hour);
  preferences.putInt("end_minute", active_end_hour.minute);
  preferences.end(); 
}

void setGracePeriod(unsigned long grace)
{
  grace_period = grace; // Define a variável global

  // Atualiza valor na NVS
  Preferences preferences;
  preferences.begin("config", false); 
  preferences.putULong("grace_period", grace_period);
  preferences.end(); 
}

void setDailyGoal(int goal)
{
  if (daily_goal == goal)
  {
    return;
  }

  daily_goal = goal;

  // Atualiza valor na NVS
  Preferences preferences;
  preferences.begin("config", false); 
  preferences.putInt("daily_goal", daily_goal);
  preferences.end(); 
}

void setContainerWeight(int weight)
{
  if (container_weight == weight)
  {
    return; 
  }

  container_weight = weight;
  Preferences preferences;
  preferences.begin("config", false); 
  preferences.putInt("cont_weight", container_weight);
  preferences.end(); 

  last_recorded_weight = 0; 
  is_container_present = false; 
  is_waiting_stability = false;
  
}

Hour getActiveStartHour()
{
  Hour active_start;

  // Consulta valor na NVS
  Preferences preferences;
  preferences.begin("config", false); 
  active_start.hour = preferences.getInt("start_hour", ACTIVE_START_HOUR.hour);
  active_start.minute = preferences.getInt("start_minute", ACTIVE_START_HOUR.minute);
  preferences.end(); 

  return active_start;
}

Hour getActiveEndHour()
{
  Hour active_end;
  
  // Consulta valor na NVS
  Preferences preferences;
  preferences.begin("config", false); 
  active_end.hour = preferences.getInt("end_hour", ACTIVE_END_HOUR.hour);
  active_end.minute = preferences.getInt("end_minute", ACTIVE_END_HOUR.minute);
  preferences.end(); 

  return active_end;
}

unsigned long getGracePeriod()
{
  unsigned long grace;

  // Consulta valor na NVS
  Preferences preferences;
  preferences.begin("config", false); 
  grace = preferences.getULong("grace_period", GRACE_PERIOD);
  preferences.end(); 

  return grace;
}

int getDailyGoal()
{
  int goal;

  // Consulta valor na NVS
  Preferences preferences;
  preferences.begin("config", false); 
  goal = preferences.getInt("daily_goal", DAILY_GOAL);
  preferences.end(); 

  return goal;
}

int getContainerWeight()
{
  int weight;
  Preferences preferences;
  preferences.begin("config", false); 
  weight = preferences.getInt("cont_weight", container_weight); // 360 default
  preferences.end(); 
  return weight;
}
/*==================================================================================*/

/* --- Definição de dados de hidratação (Partição "hydration" da NVS) --- */
/*==================================================================================*/
void setDailyConsumed(int consumed)
{
  if (daily_consumed == consumed)
  {
    return;
  }

  daily_consumed = consumed;

  // Atualiza valor na NVS
  Preferences preferences;
  preferences.begin("hydration", false);
  preferences.putInt("daily_consumed", daily_consumed);
  preferences.end(); 
}

void setLastSipTime(time_t last_sip)
{
  last_sip_time = last_sip;

  // Atualiza valor na NVS
  Preferences preferences;
  preferences.begin("hydration", false);
  preferences.putLong64("last_sip_time", (int64_t)last_sip_time);
  preferences.end(); 
}

void setLastResetDay(int last_reset)
{
  last_reset_day = last_reset;

  // Atualiza valor na NVS
  Preferences preferences;
  preferences.begin("hydration", false);
  preferences.putInt("last_reset_day", last_reset_day);
  preferences.end(); 
}

void setOfflineVolume(int volume) 
{
  offline_volume = volume;
  Preferences preferences;
  preferences.begin("hydration", false);
  preferences.putInt("offline_volume", offline_volume);
  preferences.end();
}

int getDailyConsumed()
{
  int consumed;

  // Consulta valor na NVS
  Preferences preferences;
  preferences.begin("hydration", false); 
  consumed = preferences.getInt("daily_consumed", 0);
  preferences.end(); 

  return consumed;
}

time_t getLastSipTime()
{
  int64_t last_sip;

  // Consulta valor na NVS
  Preferences preferences;
  preferences.begin("hydration", false); 
  last_sip = preferences.getLong64("last_sip_time", 0);
  preferences.end(); 

  return (time_t)last_sip;
}

int getLastResetDay()
{
  int last_reset;

  // Consulta valor na NVS
  Preferences preferences;
  preferences.begin("hydration", false); 
  last_reset = preferences.getInt("last_reset_day", 0);
  preferences.end(); 

  return last_reset;
}

int getOfflineVolume() 
{
  int volume;
  Preferences preferences;
  preferences.begin("hydration", false);
  volume = preferences.getInt("offline_volume", 0);
  preferences.end();
  return volume;
}
/*==================================================================================*/

/* --- Funções de envio de dados via WebSocket --- */
/*==================================================================================*/
void sendSip(int volume_ml)
{
  if (!webSocket.isConnected()) 
  {
    Serial.println("[Websocket] Disconnected! Sip saved for sync later.");
    setOfflineVolume(offline_volume + volume_ml);
    return;
  }

  if (offline_volume > 0) // Está conectado e existe envio pendende de agua? Junta tudo num gole só!
  {
    volume_ml += offline_volume;
    setOfflineVolume(0);
  }

  DynamicJsonDocument doc(256);
  JsonArray array = doc.to<JsonArray>();
  array.add("registrar_leitura");

  JsonObject param = array.createNestedObject();
  param["tokenAcesso"]  = TOKEN_ACESSO;
  param["quantidadeMl"] = volume_ml;

  String payload;
  serializeJson(array, payload);
  webSocket.sendTXT("42" + payload);

  Serial.printf("Sip sent: %d ml\n", volume_ml);
}

void sendContainerWeight(int empty_weight, String container_id)
{
  if (!webSocket.isConnected()) 
  {
    Serial.println("[Websocket] Disconnected! Container weight not sent.");
    return;
  }

  DynamicJsonDocument doc(256);
  JsonArray array = doc.to<JsonArray>();
  array.add("registrar_calibracao");

  JsonObject param = array.createNestedObject();
  param["tokenAcesso"]  = TOKEN_ACESSO;
  param["pesoVazioG"]   = empty_weight;
  param["recipienteId"] = container_id;

  String payload;
  serializeJson(array, payload);
  webSocket.sendTXT("42" + payload);

  Serial.printf("[Websocket] Container weight sent: %d g\n", empty_weight);
}

void sendAlert(String message)
{
  if (!webSocket.isConnected()) 
  {
    Serial.println("[Websocket] Disconnected! Alert not sent.");
    return;
  }

  DynamicJsonDocument doc(256);
  JsonArray array = doc.to<JsonArray>();
  array.add("registrar_alerta");

  JsonObject param = array.createNestedObject();
  param["tokenAcesso"] = TOKEN_ACESSO;
  param["mensagem"]    = message;

  String payload;
  serializeJson(array, payload);
  webSocket.sendTXT("42" + payload);

  Serial.printf("[Websocket] Alert sent: &s\n", message);
}

void sendCalibrationStatus(String message)
{
  if (!webSocket.isConnected())
  {
    return;
  }

  DynamicJsonDocument doc(256);
  JsonArray array = doc.to<JsonArray>();
  array.add("calibracao_status"); // O Gabriel deve ter um @SubscribeMessage('calibracao_status') no NestJS

  JsonObject param = array.createNestedObject();
  param["tokenAcesso"] = TOKEN_ACESSO;
  param["mensagem"] = message;

  String payload;
  serializeJson(array, payload);
  webSocket.sendTXT("42" + payload);
}

void sendRealTimeWeight(int current_weight)
{
  if (!webSocket.isConnected()) 
  {
    return;
  }

  DynamicJsonDocument doc(128);
  JsonArray array = doc.to<JsonArray>();
  
  // Nome do evento que o backend vai escutar
  array.add("peso_em_tempo_real");

  JsonObject param = array.createNestedObject();
  param["tokenAcesso"] = TOKEN_ACESSO;
  param["pesoAtual"] = current_weight;

  String payload;
  serializeJson(array, payload);
  
  webSocket.sendTXT("42" + payload);
}
/*==================================================================================*/

/* --- Processamento de sinais e matemática --- */
/*==================================================================================*/
float movingAverage(float reading, float samples[], int size_of_samples)
{
  for (int i = (size_of_samples - 1); i > 0; i--) 
  {
    samples[i] = samples[i - 1]; 
  }
  samples[0] = reading; 

  float sum = 0;
  for (int i = 0; i < size_of_samples; i++) 
  {
    sum += samples[i];
  }
  return sum / (float)size_of_samples; 
}

float median(float reading, float samples[], int size_of_samples)
{
  for (int i = (size_of_samples - 1); i > 0; i--) 
  {
    samples[i] = samples[i - 1];
  }
  samples[0] = reading; 

  float temp[size_of_samples]; 
  memcpy(temp, samples, size_of_samples * sizeof(float)); 
  std::sort(temp, temp + size_of_samples); 

  if (size_of_samples % 2 == 0) 
  {
    return (temp[size_of_samples / 2 - 1] + temp[size_of_samples / 2]) / 2.0; 
  }
  else 
  {
    return temp[size_of_samples / 2]; 
  }
}

float exponentialMovingAverage(float reading, int equivalent_samples) 
{
  float alpha = 2.0 / (float)(equivalent_samples + 1); 

  if (sample_ema == 0.0) 
  {
    sample_ema = reading; 
  }
  else 
  {
    sample_ema = (reading * alpha) + (sample_ema * (1.0 - alpha)); 
  }
  return sample_ema; 
}

int getConsumption(int current_weight)
{
  int consumed_volume = 0; 
  int minimum_container_weight = 15;

  static int previous_frame_weight = 0;
  if (abs(current_weight - previous_frame_weight) > NOISE_TOLERANCE) 
  {
    time_container_placed = millis(); 
  }
  previous_frame_weight = current_weight;

  if (current_weight < minimum_container_weight) // A balança está vazia?
  {
    if (is_container_present) // O recipiente estava na balança?
    {
      // O recipiente acabou de ser retirado da balança
      is_container_present = false; // Atualiza a flag para recipiente ausente
      is_waiting_stability = false; // Atualiza a flag para indicar que está estável (em Regime Permanente)
    }
  }
  else //A balança não está vazia?
  {
    if (!is_container_present) // // O recipiente não estava na balança?
    {
      // O recipiente acabou de ser colocado na balança
      is_container_present = true; // Atualiza a flag para recipiente presente
      is_waiting_stability = true; // Atualiza a flag para indicar que está aguardando estabilidade (em Regime Transitório)
      time_container_placed = millis(); 
    }

    if (is_waiting_stability) // Está aguardando estabilidade?
    {
      if (millis() - time_container_placed > SETTLING_TIME) // Atingiu o tempo de acomodação?
      {
        // Atingiu estabilidade (Regime Permanente)
        is_waiting_stability = false; 

        if (last_recorded_weight > (container_weight + NOISE_TOLERANCE)) // O ultimo peso guardado era maior que o peso do recipiente somado com a tolerância?
        {
          int weight_difference = last_recorded_weight - current_weight; 

          if (weight_difference > NOISE_TOLERANCE) 
          {
            // Se o valor for maior, provavelmente alguém encostou a mão na balança. Ignoramos.
            if (weight_difference <= 600) 
            {
              consumed_volume =  weight_difference; 

              setDailyConsumed(daily_consumed + consumed_volume);
              setLastSipTime(time(NULL));
              simulated_last_sip_time = now;

              Serial.print("Consumed water: ");
              Serial.print(consumed_volume);
              Serial.println(" ml");

              sendSip(consumed_volume);
            }
            else
            {
              Serial.println("Sip rejected! Probable weight anomaly.");
            }                 
          }
          else if (weight_difference < -NOISE_TOLERANCE) 
          {
            Serial.println("Container refilled.");
          }
        }
        last_recorded_weight = current_weight; 
      }      
    }
    else // Já estava estavel?
    {
      if (abs(current_weight - last_recorded_weight) > NOISE_TOLERANCE) 
      {
          is_waiting_stability = true;
          time_container_placed = millis();
      }
    }
  }
  return consumed_volume;
}

void calibrateScale(float known_weight)
{
  Serial.println("===============================================================================");
  Serial.println("Calibrating scale...");

  if (known_weight <= 0.0) // O peso é invalido?
  {
    sendCalibrationStatus("Erro: Peso inválido!");
    return; 
  }

  sendCalibrationStatus("Esvazie completamente a balança!");

  Serial.print("Leave the scale completely clear in: ");
  int countdown_time = 5; 
  long initial_time = millis(); 
  while (countdown_time >= 0)
  {
    webSocket.loop(); // Ouve o App durante a contagem!
    if (cancel_calibration_flag) 
    {
        sendCalibrationStatus("Calibração cancelada!");
        return;
    }

    if (millis() - initial_time >= 1000) 
    {
      Serial.print(countdown_time); Serial.print("...");
      initial_time = millis();
      countdown_time--;
    }
  }
  Serial.println();

  sendCalibrationStatus("Lendo tara. Não toque na balança..."); 

  int sample = 0;
  long offset = 0;
  while (sample < 20) 
  {
    webSocket.loop(); // Ouve o App durante a leitura do sensor!
    if (cancel_calibration_flag)
    {
      sendCalibrationStatus("Calibração cancelada!");
      return; 
    }

    if (scale.is_ready()) 
    {
      Serial.print(".");
      offset += scale.read();
      sample++;
    }
  }
  Serial.println();

  offset = (int)(offset / 20.0);
  
  scale.set_offset(offset); // Aplica o offset na RAM para o cálculo seguinte, mas a NVS continua intacta!

  String weight_message = "Coloque o peso de " + String(known_weight, 2) + "g na balança.";
  sendCalibrationStatus(weight_message); 

  Serial.print("Place the known weight on the scale in: ");
  countdown_time = 5; 
  initial_time = millis(); 
  while (countdown_time >= 0) 
  {
    webSocket.loop();
    if (cancel_calibration_flag) 
    {
        // Se cancelar aqui, restaura o Offset que estava antes de iniciarmos
        Preferences pref; pref.begin("scale", true);
        scale.set_offset(pref.getLong("offset", SCALE_OFFSET));
        pref.end();
        sendCalibrationStatus("Calibração cancelada!");
        return;
    }

    if (millis() - initial_time >= 1000) 
    {
      Serial.print(countdown_time); Serial.print("...");
      initial_time = millis();
      countdown_time--;
    }
  }
  Serial.println();

  sendCalibrationStatus("Calculando fator de escala..."); 

  sample = 0;
  double weight_read = 0;
  while (sample < 20) 
  {
    webSocket.loop();
    if (cancel_calibration_flag) 
    {
        Preferences pref; pref.begin("scale", true);
        scale.set_offset(pref.getLong("offset", SCALE_OFFSET));
        pref.end();
        sendCalibrationStatus("Calibração cancelada!");
        return;
    }

    if (scale.is_ready()) 
    {
      Serial.print(".");
      weight_read += scale.get_value(1);
      sample++;
    }
  }
  Serial.println();
  
  weight_read = weight_read / 20; 

  if (weight_read == 0.0) 
  {
    sendCalibrationStatus("Erro: Nenhum peso detectado!");
    Preferences pref; pref.begin("scale", true);
    scale.set_offset(pref.getLong("offset", SCALE_OFFSET));
    pref.end();
    
    sendCalibrationStatus("ok"); // Libera a tela do front-end mesmo se der erro
    return;
  }

  // ===================================================================
  // NOVO FINAL DA FUNÇÃO: Auto-Commit + Palavra Mágica ("ok")
  // ===================================================================

  // Calcula os novos parâmetros definitivos
  float novo_scale_divider = weight_read / known_weight; 

  // 1. APLICA IMEDIATAMENTE NA BALANÇA
  scale.set_offset(offset);
  scale.set_scale(novo_scale_divider);

  // 2. GRAVA NA MEMÓRIA PERMANENTE (NVS) NA HORA!
  Preferences preferences;
  preferences.begin("scale", false);
  preferences.putLong("offset", offset);
  preferences.putFloat("scale_divider", novo_scale_divider);
  preferences.end();

  // 3. AMNÉSIA DA MÁQUINA DE ESTADOS (Para a balança não gerar um gole falso gigante)
  last_recorded_weight = 0; 
  is_container_present = false; 
  is_waiting_stability = false;

  Serial.println("===============================================================================");
  Serial.println("[WS] Calibração concluída e salva na memória permanente!");
  Serial.print("New Offset: "); Serial.println(offset);
  Serial.print("New Divider: "); Serial.println(novo_scale_divider, 4);
  Serial.println("===============================================================================");

  // 4. MANDA MENSAGEM VISUAL DE SUCESSO PARA O APP
  sendCalibrationStatus("Calibração gravada com sucesso!"); 
  
  // Pequena pausa para o usuário ter tempo de ler a mensagem verde de sucesso na tela
  delay(1500); 
  
  // 5. MANDA A "PALAVRA MÁGICA" PARA O BACKEND FECHAR A TELA DO FRONT-END!
  sendCalibrationStatus("ok"); 
}

/* --- Função callback do WebSocket --- */
/*==================================================================================*/
void onWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) 
{
  switch (type)
  {
    case WStype_DISCONNECTED:
      Serial.println("===============================================================================");
      Serial.println("Disconnected from the NestJS server!");
      Serial.println("===============================================================================");
      break;

    case WStype_CONNECTED:
      Serial.println("===============================================================================");
      Serial.println("Connection to the NestJS has been estabilished! Waiting for the server answer.");
      Serial.println("===============================================================================");
      break;

    case WStype_TEXT:
    {
      if (length > 0 && payload[0] == '0') // O servidor abriu o túnel de conexão (cliente recebe type == '0')?
      {
        // Cliente responde que quer se conectar (cliente envia type == '40')
        Serial.println("===============================================================================");
        Serial.println("[Client -> Server] type: '40' (Requesting access)");
        Serial.println("===============================================================================");
        webSocket.sendTXT("40"); 
      }
      else if (length > 1 && payload[0] == '4' && payload[1] == '0') // O servidor aceitou a conexão (cliente recebe type == '40')?
      {
        // Conexão realizada com sucesso
        Serial.println("===============================================================================");
        Serial.println("[Server -> Client] type: '40' (Access authorized)");
        Serial.println("[Server -> Client] Awaiting commands and server pings to keep the connection alive!");
        Serial.println("===============================================================================");

        if (offline_volume > 0)
        {
          Serial.printf("[Websocket] Updating consumed volume. Volume: %d mL...\n", offline_volume);

          DynamicJsonDocument doc(256);
          JsonArray array = doc.to<JsonArray>();
          array.add("registrar_leitura");
          
          JsonObject param = array.createNestedObject();
          param["tokenAcesso"] = TOKEN_ACESSO;
          param["quantidadeMl"] = offline_volume;
          
          String payloadStr;
          serializeJson(array, payloadStr);
          webSocket.sendTXT("42" + payloadStr);
          
          setOfflineVolume(0); // Reseta a variavel de volume não enviado!
        }
      }
      else if (length > 1 && payload[0] == '4' && payload[1] == '2') // O servidor enviou um evento (cliente recebe type == '42')?
      {
        // Cliente recebe um payload JSON com comando
        char* jsonPayload = (char*)(payload + 2); // Salta o prefixo "42"
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, jsonPayload);
        
        if (!error) // Não deu erro ao desserializar o JSON?
        {
          const char* evento = doc[0];
          
          if (strcmp(evento, "comando") == 0) // O evento é do tipo "comando"?
          { 
            String command = doc[1]["comando"].as<String>(); // Recebe o comando enviado pelo servidor
            float parameter = doc[1]["parametro"].as<float>(); // Recebe qual o parâmetro do comando enviado pelo servidor

            command.toLowerCase(); // Transforma tudo em minusculas
            
            Serial.printf("Remote command received: %s\n", command.c_str());
            
            if (command == "restore")
            {
              restoreDefaults();
            }
            else if (command == "set_daily_goal")
            {
              setDailyGoal((int)parameter);
              current_state = IDLE;

              Serial.print("daily_goal updated to: ");
              Serial.println(parameter);
            }
            else if (command == "set_grace_period")
            {
              unsigned long grace_ms = (unsigned long)parameter * 60 * 1000;
              setGracePeriod(grace_ms);
            }
            else if (command == "set_container_weight")
            {
              setContainerWeight((int)parameter);              
            }
            else if (command == "set_daily_consumed")
            {
              setDailyConsumed((int)parameter);
              current_state = IDLE;

              Serial.print("daily_consumed updated to: ");
              Serial.println(parameter);

              // Sincroniza o dia de reset lógico com o dia de hoje
              struct tm time_now;
              if (getLocalTime(&time_now, 0)) 
              {
                setLastResetDay(time_now.tm_mday);
              }
            }
            else if (command == "set_active_start_hour")
            {
              Hour new_active_start_hour = {(int)parameter, 0};
              setActiveStartHour(new_active_start_hour);
            }
            else if (command == "set_active_end_hour")
            {
              Hour new_active_start_hour = {(int)parameter, 0};
              setActiveEndHour(new_active_start_hour);
            }
            else if (command == "get_container_weight")
            {
              String container_id = doc[1]["parametro"].as<String>();
              sendContainerWeight((int)filtered_weight, container_id);
            }
            else if (command == "calibrate" || command == "calibrar_balanca")
            {
              float known_weight = doc[1]["parametro"].as<float>();
              calibrateScale(known_weight);
            }
            else if (command == "cancel_calibration") 
            {
              cancel_calibration_flag = true;
            }
            else if (command == "commit_calibration") 
            {
              if (temp_scale_divider != 0.0) // Confirma se existem valores à espera
              {
                // Aplica fisicamente na balança
                scale.set_offset(temp_offset);
                scale.set_scale(temp_scale_divider);

                // Grava permanentemente na NVS
                Preferences preferences;
                preferences.begin("scale", false);
                preferences.putLong("offset", temp_offset);
                preferences.putFloat("scale_divider", temp_scale_divider);
                preferences.end();

                // Limpa os temporários
                temp_offset = 0;
                temp_scale_divider = 0.0;

                sendCalibrationStatus("Calibração gravada com sucesso!");
                Serial.println("[WS] Nova calibração salva na memória permanente!");
              }
            }
          }
        }
      }
      else if (length > 0 && payload[0] == '2') // O servidor enviou um ping (cliente recebe type == '2')?
      {
        // O cliente responde com pong (type == '3') para manter a conexão ativa
        webSocket.sendTXT("3");
      }
      break;
    case WStype_PING:
      // O cliente responde com pong (type == '3') para manter a conexão ativa
      webSocket.sendTXT("3"); // Responde o pong do Socket.io
      break;
    }
  }
}
/*==================================================================================*/

/* --- Funções de inicialização --- */
/*==================================================================================*/
void setupScale()
{
  Serial.println("===============================================================================");
  Serial.println("Initializing scale...");

  scale.begin(HX711_DT, HX711_SCK); 

  scale.power_down(); 
  delay(100);
  scale.power_up();   
  delay(100);

  Preferences preferences; 
  preferences.begin("scale", false); 

  long offset = preferences.getLong("offset", 0);          
  float scale_divider = preferences.getFloat("scale_divider", 0.0); 

  if (offset == 0 || scale_divider == 0.0) 
  {
    Serial.println("Data not found in NVS. Applying default configurations...");

    offset = SCALE_OFFSET;         
    scale_divider = SCALE_DIVIDER; 

    preferences.putLong("offset", offset);                
    preferences.putFloat("scale_divider", scale_divider); 

    Serial.println("Default configuration applied successfully!");
  }
  else
  {
    Serial.println("Data found in NVS successfully!");
  }

  preferences.end(); 

  Serial.println("Appliyng configurations to scale...");
  scale.set_offset(offset);       
  scale.set_scale(scale_divider); 

  Serial.println("Configurations applied successfully:");
  Serial.print("offset:"); Serial.println(offset);
  Serial.print("scale_divider:"); Serial.println(scale_divider);
  Serial.println("===============================================================================");
  delay(1000);
}

void setupWifi()
{
  WiFiManager wifiManager; 

  wifiManager.setConnectTimeout(120); 
  wifiManager.setConfigPortalTimeout(180); 

  Serial.println("===============================================================================");
  Serial.println("Trying to auto connect...");

  bool is_connected = wifiManager.autoConnect(AP_NAME, AP_PASSWORD); 

  if (is_connected)
  {
    Serial.println("Connected to the local Wi-Fi network!");
    Serial.print("SSID: "); Serial.println(WiFi.SSID());
    Serial.print("IP adress: "); Serial.println(WiFi.localIP());
    Serial.println("===============================================================================");

    WiFi.setAutoReconnect(true); 

    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER); 
    Serial.println("Time synchronized with NTP server!");

    delay(1000); 

    // URL com parâmetros de autenticação exigidos pelo backend
    socketIO_url = String("/socket.io/?EIO=4&transport=websocket&token=") + TOKEN_ACESSO;
    webSocket.beginSSL(WS_HOST, WS_PORT, socketIO_url.c_str()); 
    webSocket.onEvent(onWebSocketEvent);
    webSocket.setReconnectInterval(5000);

    // webSocket.enableHeartbeat(25000, 5000, 2); // ADICIONE ISSO
  }
  else
  {
    Serial.println("Connection failed. Starting in OFFLINE mode.");
    Serial.println("===============================================================================");
  }
}

void setupConfigs()
{
  Serial.println("===============================================================================");
  Serial.println("Loading system configurations from NVS...");

  // Consulta grace_period na NVS (Se estiver zerado significa que a NVS ainda não tem dados)
  Preferences preferences;
  preferences.begin("config", false); 
  unsigned long saved_grace_period = preferences.getULong("grace_period", 0);
  preferences.end(); 

  if (saved_grace_period == 0) // Não tem nenhum dado salvo na NVS?
  {
    Serial.println("Configs not found in NVS. Applying default configurations...");

    // Atualiza variavel global e NVS
    setActiveStartHour(ACTIVE_START_HOUR);
    setActiveEndHour(ACTIVE_END_HOUR);
    setGracePeriod(GRACE_PERIOD);
    setDailyGoal(DAILY_GOAL);
    setContainerWeight(0);
  }
  else
  {
    Serial.println("Configs found in NVS successfully!");

    active_start_hour = getActiveStartHour();
    active_end_hour = getActiveEndHour();
    grace_period = getGracePeriod();
    daily_goal = getDailyGoal();
    container_weight = getContainerWeight();
  }  

  Serial.println("System configurations applied successfully:");
  Serial.printf("Start Time: %02d:%02d\n", active_start_hour.hour, active_start_hour.minute);
  Serial.printf("End Time: %02d:%02d\n", active_end_hour.hour, active_end_hour.minute);
  Serial.print("Grace Period (ms): "); Serial.println(grace_period);
  Serial.print("Daily Goal (mL): "); Serial.println(daily_goal);
  Serial.print("Container Weight: "); Serial.println(container_weight);
  Serial.println("===============================================================================");
  
  delay(1000);
}

void setupHydration()
{
  Serial.println("===============================================================================");
  Serial.println("Loading hydration data from NVS...");

  daily_consumed = getDailyConsumed();
  last_sip_time = getLastSipTime();
  last_reset_day = getLastResetDay();
  offline_volume = getOfflineVolume();

  struct tm struct_last_sip_time;
  localtime_r(&last_sip_time, &struct_last_sip_time);

  Serial.println("Hydration data loaded successfully:");
  Serial.print("Daily Consumed: "); 
  Serial.print(daily_consumed); 
  Serial.println(" ml");
  Serial.printf("Offline Volume Pending: %d ml\n", offline_volume);
  Serial.print("Last sip time: "); 
  Serial.printf("%02d:%02d:%02d", struct_last_sip_time.tm_hour, struct_last_sip_time.tm_min, struct_last_sip_time.tm_sec);
  Serial.println(); 
  Serial.print("Last reset day: "); 
  Serial.println(last_reset_day); 
  Serial.println("===============================================================================");
}
/*==================================================================================*/

/* --- Gerenciamento de rede --- */
/*==================================================================================*/
void handleWifiConnection()
{
  unsigned long current_time = millis();

  if (current_time - last_wifi_check >= WIFI_CHECK_INTERVAL) 
  {
    last_wifi_check = current_time;

    if (WiFi.status() == WL_CONNECTED)
    {
      if (!wifi_was_connected)
      {
        Serial.println("[Wi-Fi] Connection successfully restored!");
        wifi_was_connected = true;
      }
    }
    else
    {
      if (wifi_was_connected)
      {
        Serial.println("[Wi-Fi] Connection lost! Trying to reconnect in the background...");
        wifi_was_connected = false;
      }
      
      WiFi.begin(); 
    }
  }
}
/*==================================================================================*/

/* --- Comandos do App e botão "en" --- */
/*==================================================================================*/
void handleHardReset()
{
  static unsigned long timePressed = 0; 

  if (digitalRead(RESTORE_DEFAULTS) == LOW)
  {
    if (timePressed == 0) 
    {
      timePressed = millis(); 
      Serial.println("[Sistema] Botao de reset pressionado. Segure por 10 segundos...");
    }
    
    if (millis() - timePressed > 10000) 
    {
      restoreDefaults(); 
      timePressed = 0; 
    }
  }
  else
  {
    if (timePressed != 0)
    {
      Serial.println("[Sistema] Reset abortado. Botao solto antes de 10 segundos.");
      timePressed = 0; 
    }
  }
}

void restoreDefaults()
{
  Serial.println("===============================================================================");
  Serial.println("Restoring default configurations and data...");

  WiFiManager wifiManager;
  wifiManager.resetSettings(); 

  Preferences preferences;

  preferences.begin("scale", false);   
  preferences.clear(); 
  preferences.end();

  preferences.begin("config", false);
  preferences.clear();
  preferences.end();

  preferences.begin("hydration", false);
  preferences.clear();
  preferences.end();

  Serial.println("Default configurations and data restored successfully!");
  Serial.println("===============================================================================");

  unsigned long countdown_timer = millis(); 
  uint8_t seconds_left = 3;                

  Serial.println("Rebooting in: ");
  while (seconds_left > 0)
  {
    if (millis() - countdown_timer >= 1000)
    {
      Serial.print(seconds_left); Serial.print("...");
      seconds_left--; 
      countdown_timer = millis(); 
    }
  }
  Serial.println();
  ESP.restart(); 
}
/*==================================================================================*/

/* --- Motor de animações visuais e sonoras (LEDs e Buzzer) --- */
/*==================================================================================*/
void turnOnLed(int index, bool r, bool g, bool b) 
{
  digitalWrite(LEDS[index].R, r); 
  digitalWrite(LEDS[index].G, g); 
  digitalWrite(LEDS[index].B, b); 
}

void turnOffAllLeds() 
{
  for (int i = 0; i < 5; i++) 
  {
    turnOnLed(i, LOW, LOW, LOW); 
  }
}

void processAnimations()
{
  if (current_playing_alert == IDLE) // O estado é IDLE?
  {
    // Se for IDLE não faz nada
    return;
  }

  unsigned long current_instant = millis(); // Instante exato desde o inicio do funcionamento do ESP

  if (is_alert_pausing) // O alerta está no intervalo entre as 3 repetições
  {
    if (current_instant - alert_pause_start >= 3000) // Passou o tempo de 3 segundos desde o ultimo alerta?
    {
      // Reinicia a animação
      is_alert_pausing = false;
      note_index = 0; // Volta ao indice zero da melodia
      current_animation_interval = 0; // Define em qual intervalo esta (Para definir quantas vezes ja tocou o alerta)
      previous_animation_time = current_instant;
    }
    return;
  }

  // Variaveis baseadas no estado da melodia
  bool r = LOW, g = LOW, b = LOW; // Cores dos LEDs
  int note_count = 0; // Quantidade de notas na melodia
  int max_repeats = 3; // Numero máximo de repetições da animacao por alerta
  const Note *current_notes = nullptr; // Vetor de notas atual

  if (current_playing_alert == GREEN)
  {
    note_count = sizeof(GREEN_NOTES) / sizeof(GREEN_NOTES[0]);
    current_notes = GREEN_NOTES;
    r = LOW;
    g = HIGH;
    b = LOW;
    max_repeats = 1;
  }
  else if (current_playing_alert == YELLOW)
  {
    note_count = sizeof(YELLOW_NOTES) / sizeof(YELLOW_NOTES[0]);
    current_notes = YELLOW_NOTES;
    r = HIGH;
    g = HIGH;
    b = LOW;
  }
  else if (current_playing_alert == RED)
  {
    note_count = sizeof(RED_NOTES) / sizeof(RED_NOTES[0]);
    current_notes = RED_NOTES;
    r = HIGH;
    g = LOW;
    b = LOW;
  }

  if (current_instant - previous_animation_time >= current_animation_interval) // sdgfdsgsd
  {
    previous_animation_time = current_instant;

    if (note_index >= note_count) // Tocou todas as melodias no ciclo atual?
    {
      alert_repeat_count++;

      if (alert_repeat_count >= max_repeats) // Repetiu a melodia o número maximo de vezes? (3 vezes para RED e YELLOW e 1 vez para GREEN)
      {
        if (!is_test_alert) // Só atualiza os tempos se NÃO for um comando do Serial
        {
          last_alert_instant = millis(); // Atualiza a variavel last_alert_instant para fazer a verificação de grace_time
          last_alert_time = time(NULL); // Para depuração, depois apagar
          simulated_last_alert_time = now;
        }

        noTone(BUZZER);
        turnOffAllLeds(); 

        current_playing_alert = IDLE; // Libera o sistema para tocar novos alertas
        is_test_alert = false; // Devolve o controle para o sistema normal
      }
      else // Ainda não tocou todas as melodias no ciclo atual?
      {
        is_alert_pausing = true;
        alert_pause_start = current_instant;

        noTone(BUZZER);

        // Acende os leds com as cores definidas
        for (int i = 0; i < 5; i++)
        {
          turnOnLed(i, r, g, b);
        }
      }
      return;
    }

    // Lê a frequência e o tempo da nota atual
    int freq = current_notes[note_index].frequency;
    int dur = current_notes[note_index].duration;

    if (freq > 0) // A frequência é maior que zero?
    {
      // Toca as melodias e executa as animações

      tone(BUZZER, freq, dur);
      turnOffAllLeds(); // Limpa para a nova animação

      if (current_playing_alert == GREEN)
      {
        // Animação iniciando no LED central
        int stage = note_index / 2;

        turnOnLed(2, r, g, b);

        if (stage >= 1)
        {
          turnOnLed(1, r, g, b);
          turnOnLed(3, r, g, b);
        }
        if (stage >= 2)
        {
          turnOnLed(0, r, g, b);
          turnOnLed(4, r, g, b);
        }
      }
      else if (current_playing_alert == YELLOW)
      {
        // Acende todos os leds
        for (int i = 0; i < 5; i++)
        {
          turnOnLed(i, r, g, b);
        }
      }
      else if (current_playing_alert == RED)
      {
        // Acende um led de cada vez da esquerda para a direita

        int active_led = note_index / 2;

        if (active_led < 5)
        {
          turnOnLed(active_led, r, g, b);
        }
      }
    }
    else // A frequência é zero?
    {
      noTone(BUZZER);
      turnOffAllLeds(); 
    }

    current_animation_interval = dur; 
    note_index++; 
  }
}

void triggerAlert(AlertState type)
{
  if (current_playing_alert != IDLE) // A animação atual não é IDLE (Deligada)?
  {
    // Não executa animação
    return;
  }

  if (type == YELLOW)
  {
    sendAlert("Alerta AMARELO. Atenção, você esta em estado de alerta para hidratação!");
  }
  else if (type == RED)
  {
    sendAlert("Alerta VERMELHO. Atenção, você esta atrasado na hidratação. Beba água!");
  }

  current_playing_alert = type;
  
  // Reinicia as variaveis de animação
  note_index = 0;                                 
  current_animation_interval = 0;                
  previous_animation_time = millis(); 
  alert_repeat_count = 0; 
  is_alert_pausing = false;
}

void handleAlerts()
{
  if (current_consumed_volume > 0) // O usuário consumiu água?
  {
    triggerAlert(GREEN);
  }

  if (current_playing_alert != IDLE) // Tem alguma animação tocando?
  {
    return; 
  }


  if (millis() - last_alert_instant < grace_period) // Está no período de carência?
  {
    switch (current_state)
    {
      case IDLE:
        turnOffAllLeds();
        break;
      case GREEN:
        for (int i = 0; i < 5; i++)
        {
          turnOnLed(i, LOW, HIGH, LOW);
        }
        break;
      case YELLOW:
        for (int i = 0; i < 5; i++)
        {
          turnOnLed(i, HIGH, HIGH, LOW);
        }
        break;
      case RED:
        for (int i = 0; i < 5; i++)
        {
          turnOnLed(i, HIGH, LOW, LOW);
        }
        break;
    }
  }
  else // Não está no período de carência?
  {
    if (current_state == YELLOW || current_state == RED)
    {
      triggerAlert(current_state);
    }
    else if (current_state == GREEN)
    {
      for (int i = 0; i < 5; i++)
      {
        turnOnLed(i, LOW, HIGH, LOW);
      }
    }
    else if (current_state == IDLE)
    {
      turnOffAllLeds();
    }

  }

  
}
/*==================================================================================*/

/* --- Controle de Tempo e Datas --- */
/*==================================================================================*/
void handleDayChange()
{
  struct tm time_now;

  if (!getLocalTime(&time_now, 0)) // O relógio ainda não está sincronizado com NTP?
  {
    // Se não estiver, retorna sem fazer nada
    return; 
  }

  bool apply_reset = false;

  // Verifica EXCLUSIVAMENTE se houve mudança de data no calendário
  if (last_reset_day != time_now.tm_mday) 
  {
    apply_reset = true; 
  }

  if (apply_reset) // A flag de reset foi acionada?
  {
    Serial.println("===============================================================================");
    Serial.println("New date detected! Reseting consumption...");

    current_state = IDLE;
    deficit = 0;

    daily_consumed = 0;
    last_reset_day = time_now.tm_mday; // Atualiza o ultimo reset para o dia atual
    offline_volume = 0;

    Preferences preferences;
    preferences.begin("hydration", false);
    preferences.putInt("daily_consumed", 0);
    preferences.putInt("last_reset_day", last_reset_day);
    preferences.putInt("offline_volume", 0);
    preferences.end();

    Serial.println("===============================================================================");
  }
  
  was_active = is_active;
}
/*==================================================================================*/

/* --- Gerenciamento da Máquina de Estados (FSM) --- */
/*==================================================================================*/
void handleFiniteStateMachine(int consumed_now) // Quando virar o dia (Terminar o horario ativo, deve zerar o deficit e voltar à IDLE)
{
  struct tm time_now; 
  localtime_r(&now, &time_now); // Converte o horario para uma struct

  int current_mins = time_now.tm_hour * 60 + time_now.tm_min;
  int start_mins = active_start_hour.hour * 60 + active_start_hour.minute;
  int end_mins = active_end_hour.hour * 60 + active_end_hour.minute;

  // Serial.print("current_mins: "); Serial.print(current_mins); Serial.print(", ");
  // Serial.print("start_mins: "); Serial.print(start_mins); Serial.print(", ");
  // Serial.print("end_mins: "); Serial.print(end_mins); Serial.println(", ");

  is_active = [&]() -> bool {  
    if (start_mins <= end_mins) // Janela padrão (ex: 17:00 às 23:59)
    {      
      return (current_mins >= start_mins && current_mins <= end_mins);
    } 
    else // Janela que passa da meia-noite (ex: 22:00 às 06:00)
    {      
      return (current_mins >= start_mins || current_mins <= end_mins);
    }
  }(); // Está no horário ativo?

  deficit = [&]() -> int {
    if (!is_active) 
    {
      return 0; // Se não estiver no horário, zera o déficit.
    }

    int total_active_mins = 0; 

    if (start_mins <= end_mins) // Janela padrão (ex: 17:00 às 23:59)
    {      
      total_active_mins = end_mins - start_mins; 
    } 
    else // Janela que passa da meia-noite (ex: 22:00 às 06:00)
    {      
      total_active_mins = (1440 - start_mins) + end_mins;
    }

    int mins_passed = 0; 

    if (start_mins <= end_mins) // Janela padrão (ex: 17:00 às 23:59) e o horario ja virou a noite
    {      
      mins_passed = current_mins - start_mins;
    } 
    else // Janela que passa da meia-noite (ex: 22:00 às 06:00)
    {      
      if (current_mins <= end_mins) // Horario passou das 23:59
      {
        mins_passed = (1440 - start_mins) + current_mins;
      }    
      else // Horário não passou das 23:59
      {
        mins_passed = current_mins - start_mins;
      }  
    } 

    if (total_active_mins == 0) // Evita erro de divisão por zero
    {
      return 0;
    }

    current_ideal_volume = (int)(((float)daily_goal / total_active_mins) * mins_passed);
    int calculated_deficit = current_ideal_volume - daily_consumed;

    return (calculated_deficit < 0) ? 0 : calculated_deficit; // Trava o déficit em 0 para não exibir números negativos (superávit)
  }(); // Deficit de hidratacao


  switch (current_state)
  {
    case IDLE:
      if (is_active) // Está no horário ativo?
      {
        if (deficit < 50)
        {
          current_state = GREEN;
          return;
        }
        else if (deficit >= 50 && deficit < 300)
        {
          current_state = YELLOW;
          return;
        }
        else if (deficit >= 300)
        {
          current_state = RED;
          return;
        }
      }
      else
      {
        current_state = IDLE;
        return;
      }
      break;
    case GREEN:
      if (is_active) // Está no horário ativo?
      {
        if (consumed_now > 0) // Um gole foi ingerido ?
        {
          current_state = GREEN; // Continua no verde
          return;
        }  
        else
        {
          if (deficit < 50)
          {
            current_state = GREEN;
            return;
          }
          else if (deficit >= 50 && deficit < 300)
          {
            current_state = YELLOW;
            return;
          }
          else if (deficit >= 300)
          {
            current_state = RED;
            return;
          }
        }      
      }
      else
      {
        current_state = IDLE;
        return;
      }
      break;
    case YELLOW:
      if (is_active) // Está no horário ativo?
      {
        if (consumed_now > 0) // Um gole foi ingerido ?
        {
          current_state = GREEN; // Vai pro verde
          return;
        }  
        else
        {
          if (deficit >= 300)
          {
            current_state = RED;
            return;
          }
          else
          {
            return; // Não acontece nada (Permanece YELLOW)
          }
        }      
      }
      else
      {
        current_state = IDLE;
        return;
      }
      break;
    case RED:
      if (is_active) // Está no horário ativo?
      {
        if (consumed_now > 0) // Um gole foi ingerido ?
        {
          current_state = GREEN; // Vai pro verde
          return;
        }  
        else
        {
          return; // Não acontece nada (Permanece RED)
        }  
      }
      else
      {
        current_state = IDLE;
        return;
      }
      break;
  }
}
/*==================================================================================*/

/* --- Testes e depuracao --- */
/*==================================================================================*/
void handleSerialCommands() 
{
  if (Serial.available() > 0) 
  {
    char cmd = Serial.read(); 
    
    if (cmd == 'G' || cmd == 'g') 
    {
      Serial.println("[TESTE] Disparando Alerta VERDE!");
      is_test_alert = true; // Avisa que é um teste
      triggerAlert(GREEN); 
    }
    else if (cmd == 'Y' || cmd == 'y') 
    {
      Serial.println("[TESTE] Disparando Alerta AMARELO!");
      is_test_alert = true; // Avisa que é um teste
      triggerAlert(YELLOW);
    }
    else if (cmd == 'R' || cmd == 'r') 
    {
      Serial.println("[TESTE] Disparando Alerta VERMELHO!");
      is_test_alert = true; // Avisa que é um teste
      triggerAlert(RED);
    }
    else if (cmd == 'P' || cmd == 'p') 
    {
      Serial.println("[TESTE] Forçando a parada das animações!");
      current_playing_alert = IDLE;
      is_test_alert = false; // Reseta a flag ao cancelar
      noTone(BUZZER);
      turnOffAllLeds();
    }
    else if (cmd == 'A' || cmd == 'a') 
    {
      time_acceleration_active = !time_acceleration_active;
      
      if (time_acceleration_active) 
      {
        Serial.println("\n===============================================================");
        Serial.printf("Tempo Acelerado ATIVADO! (Fator: %d segs por tick)\n", acceleration_factor);
        Serial.println("===============================================================\n");
        
        real_grace_period = grace_period; // Salva o original
        grace_period = 5000;              // Encolhe a carência para 5 segundos no teste             
        last_accel_tick = millis();
      } 
      else 
      {
        Serial.println("\n===============================================================");
        Serial.println("Tempo Acelerado DESATIVADO!");
        Serial.println("===============================================================\n");
        
        grace_period = real_grace_period; // Restaura o original
        simulated_time_offset = 0;        // Zera o tempo falso
      }
    }
    else if (cmd == 'H' || cmd == 'h') 
    {
      restoreDefaults();
    }
  }
}

void printFiniteStateMachineData () // Printa os dados da máquina de estados
{
  char *state; // Cria uma "string" para o estado da máquina

  switch (current_state)
  {
    case IDLE:
      state = "IDLE";
      break;
    case GREEN:
      state = "GREEN";
      break;
    case YELLOW:
      state = "YELLOW";
      break;
    case RED:
      state = "RED";
      break;
  }

  Serial.print("current_state:");
  Serial.print(state); 
  Serial.print(", ");

  Serial.print("deficit:");
  Serial.print(deficit); 
  Serial.print(", ");

  Serial.print("current_ideal_volume:");
  Serial.print(current_ideal_volume); 
  Serial.print(", ");

  Serial.print("daily_consumed:");
  Serial.print(daily_consumed); 
  Serial.print(", ");
}

void applyTimeAcceleration() 
{
  now = time(NULL); // Puxa o tempo real do RTC do ESP32
  time_t real_time = now; 
  
  if (time_acceleration_active) // A aceleração de tempo está ativa?
  {
    unsigned long current_millis = millis();
    
    if (current_millis - last_accel_tick >= 1000) 
    {
      simulated_time_offset += acceleration_factor; // Adiciona x segundos irreais a cada segundo real
      last_accel_tick = current_millis;
      
      struct tm tm_sim;
      accelerated_time = real_time + simulated_time_offset;
      localtime_r(&accelerated_time, &tm_sim);
    }
    
    now += simulated_time_offset; // A FSM e a depuração olham apenas para o tempo distorcido
  }
}
/*==================================================================================*/

void setup() 
{
  Serial.begin(115200);
  delay(1000);

  pinMode(RESTORE_DEFAULTS, INPUT_PULLUP); 
  pinMode(BUZZER, OUTPUT);                 
  for (int i = 0; i < 5; i++) 
  {
    pinMode(LEDS[i].R, OUTPUT);
    pinMode(LEDS[i].G, OUTPUT);
    pinMode(LEDS[i].B, OUTPUT);

    turnOnLed(i, LOW, LOW, LOW); 
  }

  setupScale(); 
  setupWifi(); 
  setupConfigs(); 
  setupHydration(); 

  // Variaveis para simular tempo acelerado
  simulated_last_sip_time = last_sip_time;
  simulated_last_alert_time = last_alert_time;

  last_alert_instant = millis() - grace_period; // Força o sistema a inicilizar um alerta se os estados forem YELLOW ou RED
}

void loop() 
{
  // Nao apagar mesmo se estiver comentado!
  now = time(NULL); // Atualiza o horario (Tem que fazer isso antes de qualquer outra coisa)

  // Depuração e testes
  applyTimeAcceleration(); // Acelera o tempo
  handleSerialCommands(); // Gerencia comandos recebidos pela porta Serial

  // Gerenciamento de rede e hardware
  handleWifiConnection(); // Gerencia Conexão WiFi
  handleHardReset(); // Gerencia botão "en" para restaurar dados de fabrica

  webSocket.loop(); // Mantém o WebSocket vivo e escuta comandos remotos

  // Envia um "Estou Online" para o backend a cada 30 segundos
  if (millis() - last_ws_ping > 30000) 
  { 
    last_ws_ping = millis();
    
    if (webSocket.isConnected()) 
    {
      DynamicJsonDocument doc(128);
      JsonArray array = doc.to<JsonArray>();
      array.add("ping_dispositivo"); // Nome do evento no NestJS

      JsonObject param = array.createNestedObject();
      param["tokenAcesso"] = TOKEN_ACESSO;

      String payload;
      serializeJson(array, payload);
      webSocket.sendTXT("42" + payload);
      
      // Descomente para debugar se necessário
      // Serial.println("[WS] Ping de status online enviado ao backend.");
    }
  }

  // Gerenciamento da FSM
  handleFiniteStateMachine(current_consumed_volume); // Gerencia os estados da maquina de estados

  // Gerenciamento de mudança de datas
  handleDayChange(); // Gerencia mudança de datas (Tem que ser depois de handleFiniteStateMachine por que precisa da variavel is_active atualizada)

  // Gerenciamento das animações
  handleAlerts(); // Gerencia os alertas
  processAnimations(); // Processa as animaçoes (Tem quer depois de handleFiniteStateMachine por que precisa do estado atualizado)

  if(scale.is_ready())
  {
    weight = scale.get_units(1);     
    median_weight = median(weight, samples_median, NUM_SAMPLES_MEDIAN);
    filtered_weight = exponentialMovingAverage(median_weight, NUM_SAMPLES_MEDIAN); 

    current_consumed_volume = getConsumption((int)filtered_weight); // Calcula o volume consumido (ou não)

    if (millis() - last_realtime_send > 1000) // 1000ms = 1s
    {
      last_realtime_send = millis();
      
      sendRealTimeWeight((int)filtered_weight);
    }

    // Tempo real sem aceleracao de tempo
    time_t real_time_now = time(NULL); 
    struct tm struct_real_time; 
    localtime_r(&real_time_now, &struct_real_time);

    // Tempo acelerado
    struct tm struct_fsm_time;
    localtime_r(&now, &struct_fsm_time);

    // Horario do ultimo gole
    time_t display_sip = time_acceleration_active ? simulated_last_sip_time : last_sip_time;
    struct tm struct_last_sip_time; 
    localtime_r(&display_sip, &struct_last_sip_time);

    // Horario do ultimo alerta
    time_t display_alert = time_acceleration_active ? simulated_last_alert_time : last_alert_time;
    struct tm struct_last_alert_time; 
    localtime_r(&display_alert, &struct_last_alert_time);
    

  
  
  //   // // Instante do ultimo alerta
  //   // Serial.print("last_alert_instant:");
  //   // Serial.print(last_alert_instant); 
  //   // Serial.print(", ");    
    
  //   // // Instante exato desde o inicio do funcionamento do ESP32
  //   // Serial.print("instant:");
  //   // Serial.print(millis()); 
  //   // Serial.print(", ");
    
  //   // // Horario do ultimo gole
  //   // Serial.print("last_sip_time:");
  //   // if (last_sip_time == 0) Serial.print("00:00:00");
  //   // else Serial.printf("%02d:%02d:%02d", struct_last_sip_time.tm_hour, struct_last_sip_time.tm_min, struct_last_sip_time.tm_sec); 
  //   // Serial.print(", ");

  //   // Horario do ultimo gole
  //   Serial.print("last_sip_time:");
  //   Serial.printf("%02d:%02d:%02d", struct_last_sip_time.tm_hour, struct_last_sip_time.tm_min, struct_last_sip_time.tm_sec);
  //   Serial.print(", ");

  //   // Printa os dados da maquina de estados
  //   printFiniteStateMachineData();

    // Peso bruto
    Serial.print("weight:");
    Serial.print(weight, 2); 
    Serial.print(", ");

    // Peso filtrado
    Serial.print("filtered_weight:");
    Serial.print(filtered_weight, 2);   
    Serial.print(", "); 

    char *state; // Cria uma "string" para o estado da máquina

    switch (current_state)
    {
      case IDLE:
        state = "IDLE";
        break;
      case GREEN:
        state = "GREEN";
        break;
      case YELLOW:
        state = "YELLOW";
        break;
      case RED:
        state = "RED";
        break;
    }

    Serial.print("current_state:");
    Serial.print(state); 
    Serial.print(", ");

    // Periodo de carencia
    Serial.print("grace_period:");
    Serial.print(grace_period / 1000 / 60); 
    Serial.print(", ");

    // // Horario de inicio
    // Serial.print("active_start_hour:");
    // Serial.print(active_start_hour.hour); 
    // Serial.print(", ");

    // // Horario de inicio
    // Serial.print("active_end_hour:");
    // Serial.print(active_end_hour.hour); 
    // Serial.print(", ");

    // Peso do recipiente
    Serial.print("container_weight:");
    Serial.print(container_weight); 
    Serial.print(", ");

    // Horario atual
    Serial.print("time:");
    Serial.printf("%02d:%02d:%02d", struct_real_time.tm_hour, struct_real_time.tm_min, struct_real_time.tm_sec);
    Serial.print(", ");

    // Horário acelerado
    Serial.print("accel_time:");
    Serial.printf("%02d:%02d:%02d", struct_fsm_time.tm_hour, struct_fsm_time.tm_min, struct_fsm_time.tm_sec); 
    Serial.print(", ");

    // Horario do ultimo alerta
    Serial.print("last_alert_time:");
    if (last_alert_time == 0) Serial.print("00:00:00");
    else Serial.printf("%02d:%02d:%02d", struct_last_alert_time.tm_hour, struct_last_alert_time.tm_min, struct_last_alert_time.tm_sec); 
    Serial.print(", ");

    // Meta diaria
    Serial.print("daily_goal:");
    Serial.print(daily_goal); 
    Serial.print(", ");

    // Valor diario consumido
    Serial.print("daily_consumed:");
    Serial.print(daily_consumed); 
    Serial.print(", ");

    // Volume ideal
    Serial.print("current_ideal_volume:");
    Serial.print(current_ideal_volume); 
    Serial.print(", ");

    // Deficit
    Serial.print("deficit:");
    Serial.print(deficit); 
    Serial.println();
  }
}

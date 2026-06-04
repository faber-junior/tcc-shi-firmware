#include "HX711.h"             // Biblioteca para comunicação com o conversor A/D da balança
#include <Preferences.h>       // Biblioteca padrão para leitura/escrita na memória flash (NVS)
#include <WiFi.h>              // Biblioteca padrão para conexão WiFi
#include <WiFiManager.h>       // Biblioteca para gerenciamento de rede (Portal Cativo)
#include "time.h"              // Biblioteca padrão do C++ para lidar com relógio e data
#include <WebSocketsClient.h>  // Biblioteca para gerenciar clientes WebSocket (by Markus Sattler)
// #include <SocketIOclient.h>    // Biblioteca oficial de Socket.io para ESP32
#include <ArduinoJson.h>       // Biblioteca para lidar com JSONs
#include <HTTPClient.h>        // Biblioteca para enviar requisições POST REST


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
const Hour ACTIVE_START_HOUR = {0, 1}; // Horário de inicio das atividades da balança
const Hour ACTIVE_END_HOUR = {23, 58};// Horário de fim das atividades da balança
const unsigned long GRACE_PERIOD = 900000; // Periodo de carência (15 minutos em ms)
// const unsigned long GRACE_PERIOD = 60000; // Periodo de carência (1 minutos em ms)

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
const int NOISE_TOLERANCE = 2; // Goles menores que 2mL são considerados ruído e ignorados (Porque podem estar proximo da borda entre um e outro ex: 56.9 - 57.1)
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
unsigned long last_ws_ping = 0; // Temporizador para o Heartbeat do Socket.IO

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
int container_weight = 360; // Mudar para zero depois de integrar ao web app

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
unsigned int acceleration_factor = 60;
time_t accelerated_time;
time_t simulated_last_sip_time = 0;
time_t simulated_last_alert_time = 0;
static unsigned long last_heap_print = 0;
unsigned long last_http_ping = 0;

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
  daily_goal = goal;

  // Atualiza valor na NVS
  Preferences preferences;
  preferences.begin("config", false); 
  preferences.putInt("daily_goal", daily_goal);
  preferences.end(); 
}

void setContainerWeight(int weight)
{
  container_weight = weight;
  Preferences preferences;
  preferences.begin("config", false); 
  preferences.putInt("cont_weight", container_weight);
  preferences.end(); 
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
/*==================================================================================*/

/* --- Funções HTTP REST (Envio de Leitura e Calibração para o NestJS) --- */
/*==================================================================================*/
void enviarGoleBackend(int quantidadeMl)
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[HTTP] WiFi desconectado. Gole não enviado.");
    return;
  }

  HTTPClient http;
  String url = "https://" + String(WS_HOST) + "/dispositivos/leitura";

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<128> doc;
  doc["tokenAcesso"] = TOKEN_ACESSO;
  doc["quantidadeMl"] = quantidadeMl;

  String payload;
  serializeJson(doc, payload);

  int httpCode = http.POST(payload);
  Serial.printf("[HTTP] POST Gole (%d mL) -> Code: %d\n", quantidadeMl, httpCode);
  http.end();
}

// void enviarLeituraCalibracaoBackend(int pesoVazioG)
// {
//   if (WiFi.status() != WL_CONNECTED) {
//     Serial.println("[HTTP] WiFi desconectado. Calibração não enviada.");
//     return;
//   }

//   HTTPClient http;
//   String url = "https://" + String(WS_HOST) + "/dispositivos/leitura-calibracao";

//   http.begin(url);
//   http.addHeader("Content-Type", "application/json");

//   StaticJsonDocument<128> doc;
//   doc["tokenAcesso"] = TOKEN_ACESSO;
//   doc["pesoVazioG"] = pesoVazioG;

//   String payload;
//   serializeJson(doc, payload);

//   int httpCode = http.POST(payload);
//   Serial.printf("[HTTP] POST Calibração (%d g) -> Code: %d\n", pesoVazioG, httpCode);
//   http.end();
// }
void enviarLeituraCalibracaoBackend(int pesoVazioG, String recipienteId)
{
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  char url[128];
  snprintf(url, sizeof(url), "https://%s/dispositivos/leitura-calibracao", WS_HOST);

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<256> doc;
  doc["tokenAcesso"]  = TOKEN_ACESSO;
  doc["pesoVazioG"]   = pesoVazioG;
  doc["recipienteId"] = recipienteId;

  String payload;
  serializeJson(doc, payload);

  int httpCode = http.POST(payload);
  Serial.printf("[HTTP] POST leitura-calibracao (%dg) -> %d\n", pesoVazioG, httpCode);
  http.end();
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

  if (current_weight < container_weight) // A balança está vazia?
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

    if (is_waiting_stability && (millis() - time_container_placed > SETTLING_TIME)) // Está aguardando estabilidade e atingiu o tempo de acomodação
    {
      // Atingiu estabilidade (Regime Permanente)
      is_waiting_stability = false; 

      if (last_recorded_weight > (container_weight + NOISE_TOLERANCE)) // O ultimo peso guardado era maior que o peso do recipiente somado com a tolerância?
      {
        int weight_difference = last_recorded_weight - current_weight; 

        if (weight_difference > NOISE_TOLERANCE) 
        {
          consumed_volume =  weight_difference; 

          setDailyConsumed(daily_consumed + consumed_volume);
          setLastSipTime(time(NULL));
          simulated_last_sip_time = now;

          Serial.print("Consumed water: ");
          Serial.print(consumed_volume);
          Serial.println(" ml");

          enviarGoleBackend(consumed_volume);
        }
        else if (weight_difference < -NOISE_TOLERANCE) 
        {
          Serial.println("Container refilled.");
        }
      }
      last_recorded_weight = current_weight; 
    }
  }
  return consumed_volume;
}
/*==================================================================================*/

/* --- Lógica RAW do Socket.io v4 --- */
/*==================================================================================*/
// void onWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) 
// {
//   switch (type)
//   {
//     case WStype_DISCONNECTED:
//       Serial.println("===============================================================================");
//       Serial.println("[WS] Desconectado do servidor NestJS!");
//       Serial.println("===============================================================================");
//       break;

//     case WStype_CONNECTED:
//       Serial.println("===============================================================================");
//       Serial.println("[WS] Conectado na camada TCP. Cumprimentando o Socket.io...");
//       // O Socket.IO v4 exige o envio do pacote "40" para oficializar o Handshake
//       webSocket.sendTXT("40");
//       break;

//     case WStype_TEXT:
//       // Pacote Engine.io "0" = Sessão aberta. O servidor está pronto.
//       if (length > 0 && payload[0] == '0') 
//       {
//         Serial.println("[WS] Sessão aberta! Validando dispositivo...");
        
//         DynamicJsonDocument doc(256);
//         JsonArray array = doc.to<JsonArray>();
//         array.add("registrar_dispositivo"); 
        
//         JsonObject param = array.createNestedObject();
//         param["token"] = TOKEN_ACESSO; // O backend espera estritamente a chave "token"
        
//         String jsonStr;
//         serializeJson(array, jsonStr); 
        
//         // "42" significa Mensagem (4) de Evento (2)
//         webSocket.sendTXT("42" + jsonStr); 
//       }
//       // Pacote "40" de volta = Registro autenticado pelo backend
//       else if (length > 1 && payload[0] == '4' && payload[1] == '0') 
//       {
//         Serial.println("[WS] Autenticado com sucesso no Gateway!");
//         Serial.println("===============================================================================");
//       }
//       // Pacote "42" = Recebemos um Comando Remoto (Ex: CALIBRAR)
//       else if (length > 1 && payload[0] == '4' && payload[1] == '2') 
//       {
//         char* jsonPayload = (char*)(payload + 2); // Salta o prefixo "42"
//         StaticJsonDocument<256> doc;
//         DeserializationError error = deserializeJson(doc, jsonPayload);
        
//         if (!error) {
//           const char* evento = doc[0];
          
//           if (strcmp(evento, "comando") == 0) { 
//             String comandoReal = doc[1]["comando"].as<String>();
//             float parametro = doc[1]["parametro"].as<float>();
            
//             Serial.printf("[WS] Ordem Remota: %s\n", comandoReal.c_str());
            
//             if (comandoReal == "CALIBRAR" || comandoReal == "calibrate") {
//               Serial.println("[WS] A executar Tara via remota...");
//               if (scale.is_ready()) {
//                 float weight = scale.get_units(5); 
//                 int peso_vazio = (int)weight;
//                 setContainerWeight(peso_vazio);    
//                 last_recorded_weight = 0;          
                
//                 Serial.printf("[Calibração] Peso salvo: %dg\n", peso_vazio);
//                 enviarLeituraCalibracaoBackend(peso_vazio); 
//               }
//             }
//             else if (comandoReal == "ZERAR_META") {
//               Serial.println("[WS] Resetando consumo diário...");
//               setDailyConsumed(0);
//               setLastSipTime(0);
//               simulated_last_sip_time = 0; 
//               current_state = IDLE;
//             }
//             else if (comandoReal == "set_goal") {
//               setDailyGoal((int)parametro);
//             }
//             else if (comandoReal == "restore") {
//               restoreDefaults();
//             }
//             else if (comandoReal == "alerta_hidratacao") {
//               triggerAlert(YELLOW);
//             }
//             else if (comandoReal == "set_horario_acordar") {
//               Hour novo_horario = {(int)parametro, 0};
//               setActiveStartHour(novo_horario);
//             }
//             else if (comandoReal == "set_horario_dormir") {
//               Hour novo_horario = {(int)parametro, 0};
//               setActiveEndHour(novo_horario);
//             }
//             else if (comandoReal == "set_grace_period") {
//               unsigned long grace_ms = (unsigned long)parametro * 60 * 1000;
//               setGracePeriod(grace_ms);
//             }
//           }
//         }
//       }
//       // Pacote "3" = Pong do Servidor confirmando que a nossa placa está viva
//       else if (length > 0 && payload[0] == '3') {
//         // Ping respondido, túnel saudável!
//       }
//       break;
//   }
// }
// void onWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) 
// {
//   switch (type)
//   {
//     case WStype_DISCONNECTED:
//       Serial.println("\n[WS ERRO] =========================================");
//       Serial.println("[WS ERRO] DESCONECTADO DO SERVIDOR NESTJS!");
//       // Tenta imprimir o motivo se o servidor o tiver enviado
//       if (length > 0) {
//         String motivo = String((char*)payload);
//         Serial.println("[WS ERRO] Motivo recebido: " + motivo);
//       }
//       Serial.println("[WS ERRO] =========================================\n");
//       break;

//     case WStype_CONNECTED:
//       Serial.println("\n[WS] ==============================================");
//       Serial.println("[WS] Conexão TCP estabelecida! Aguardando o servidor falar...");
//       Serial.println("[WS] ==============================================\n");
//       break;

//     case WStype_TEXT:
//     {
//       // A VISÃO DE RAIO-X: Imprime tudo o que chega do NestJS antes de processar!
//       String rawMsg = String((char*)payload);
//       Serial.println("[WS SERVER DIZ] -> " + rawMsg);

//       // Passo 1: O Servidor abriu as portas (Pacote "0")
//       if (length > 0 && payload[0] == '0') 
//       {
//         Serial.println("[WS CLIENT DIZ] -> 40 (Solicitando entrada...)");
//         webSocket.sendTXT("40"); 
//       }
//       // Passo 2: O Servidor autorizou a entrada (Pacote "40")
//       else if (length > 1 && payload[0] == '4' && payload[1] == '0') 
//       {
//         Serial.println("[WS] Autenticado! Enviando dados de registro...");
        
//         DynamicJsonDocument doc(256);
//         JsonArray array = doc.to<JsonArray>();
//         array.add("registrar_dispositivo"); 
        
//         JsonObject param = array.createNestedObject();
//         param["token"] = TOKEN_ACESSO; 
        
//         String jsonStr;
//         serializeJson(array, jsonStr); 
        
//         String pacoteFinal = "42" + jsonStr;
//         Serial.println("[WS CLIENT DIZ] -> " + pacoteFinal);
//         webSocket.sendTXT(pacoteFinal);
//       }
//       // Passo 3: Recebemos um Comando Remoto (Pacote "42")
//       else if (length > 1 && payload[0] == '4' && payload[1] == '2') 
//       {
//         char* jsonPayload = (char*)(payload + 2); // Salta o prefixo "42"
//         StaticJsonDocument<256> doc;
//         DeserializationError error = deserializeJson(doc, jsonPayload);
        
//         if (!error) {
//           const char* evento = doc[0];
          
//           if (strcmp(evento, "comando") == 0) { 
//             String comandoReal = doc[1]["comando"].as<String>();
//             float parametro = doc[1]["parametro"].as<float>();
            
//             Serial.printf("[WS APP] Ordem Remota Executada: %s\n", comandoReal.c_str());
            
//             if (comandoReal == "CALIBRAR" || comandoReal == "calibrate") {
//               if (scale.is_ready()) {
//                 float weight = scale.get_units(5); 
//                 int peso_vazio = (int)weight;
//                 setContainerWeight(peso_vazio);    
//                 last_recorded_weight = 0;          
//                 enviarLeituraCalibracaoBackend(peso_vazio); 
//               }
//             }
//             else if (comandoReal == "ZERAR_META") {
//               setDailyConsumed(0);
//               setLastSipTime(0);
//               simulated_last_sip_time = 0; 
//               current_state = IDLE;
//             }
//             else if (comandoReal == "set_goal") {
//               setDailyGoal((int)parametro);
//             }
//             else if (comandoReal == "restore") {
//               restoreDefaults();
//             }
//             else if (comandoReal == "alerta_hidratacao") {
//               triggerAlert(YELLOW);
//             }
//             else if (comandoReal == "set_horario_acordar") {
//               Hour novo_horario = {(int)parametro, 0};
//               setActiveStartHour(novo_horario);
//             }
//             else if (comandoReal == "set_horario_dormir") {
//               Hour novo_horario = {(int)parametro, 0};
//               setActiveEndHour(novo_horario);
//             }
//             else if (comandoReal == "set_grace_period") {
//               unsigned long grace_ms = (unsigned long)parametro * 60 * 1000;
//               setGracePeriod(grace_ms);
//             }
//           }
//         }
//       }
//       // Passo 4: O NestJS enviou um Ping ("2") para saber se a balança está viva!
//       else if (length > 0 && payload[0] == '2') 
//       {
//         // Se não respondermos com "3" rapidamente, o NestJS corta a conexão!
//         // Serial.println("[WS CLIENT DIZ] -> 3 (Pong! Estou viva!)");
//         webSocket.sendTXT("3");
//       }
//       // O NestJS respondeu ao nosso Ping ("3")
//       else if (length > 0 && payload[0] == '3') 
//       {
//         // Ping aceite pelo servidor.
//       }
//       break;
//     }
//   }
// }
void onWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) 
{
  switch (type)
  {
    case WStype_DISCONNECTED:
      Serial.println("\n[WS ERRO] =========================================");
      Serial.println("[WS ERRO] DESCONECTADO DO SERVIDOR NESTJS!");
      Serial.println("[WS ERRO] =========================================\n");
      break;

    case WStype_CONNECTED:
      Serial.println("\n[WS] ==============================================");
      Serial.println("[WS] Conexão TCP estabelecida! Aguardando o servidor falar...");
      Serial.println("[WS] ==============================================\n");
      break;

    case WStype_TEXT:
    {
      // Passo 1: O Servidor abriu as portas (Pacote "0")
      if (length > 0 && payload[0] == '0') 
      {
        Serial.println("[WS CLIENT DIZ] -> 40 (Solicitando entrada...)");
        webSocket.sendTXT("40"); 
      }
      // Passo 2: O Servidor autorizou a entrada (Pacote "40")
      else if (length > 1 && payload[0] == '4' && payload[1] == '0') 
      {
        Serial.println("[WS] Autenticado! (O NestJS já nos registou automaticamente pela URL).");
        Serial.println("[WS] A aguardar comandos e Pings do servidor para manter a conexão viva...");
        Serial.println("===============================================================================");
        
        // A MÁGICA É AQUI: Já não enviamos o evento de registo ("42").
        // Desta forma, não acionamos o ValidationPipe do NestJS e a conexão não cai!
      }
      // Passo 3: Recebemos um Comando Remoto do NestJS (Pacote "42")
      else if (length > 1 && payload[0] == '4' && payload[1] == '2') 
      {
        char* jsonPayload = (char*)(payload + 2); // Salta o prefixo "42"
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, jsonPayload);
        
        if (!error) {
          const char* evento = doc[0];
          
          if (strcmp(evento, "comando") == 0) { 
            String comandoReal = doc[1]["comando"].as<String>();
            float parametro = doc[1]["parametro"].as<float>();
            
            Serial.printf("[WS APP] Ordem Remota Executada: %s\n", comandoReal.c_str());
            
            if (comandoReal == "CALIBRAR" || comandoReal == "calibrate") {
              String recipiente_id = doc[1]["parametro"].as<String>();
              setContainerWeight((int)filtered_weight);   
              last_recorded_weight = 0; 
              enviarLeituraCalibracaoBackend((int)filtered_weight, recipiente_id);
              
            }
            else if (comandoReal == "ZERAR_META") {
              setDailyConsumed(0);
              setLastSipTime(0);
              simulated_last_sip_time = 0; 
              current_state = IDLE;
            }
            else if (comandoReal == "set_goal") {
              setDailyGoal((int)parametro);
            }
            else if (comandoReal == "restore") {
              restoreDefaults();
            }
            else if (comandoReal == "alerta_hidratacao") {
              triggerAlert(YELLOW);
            }
            else if (comandoReal == "set_horario_acordar") {
              Hour novo_horario = {(int)parametro, 0};
              setActiveStartHour(novo_horario);
            }
            else if (comandoReal == "set_horario_dormir") {
              Hour novo_horario = {(int)parametro, 0};
              setActiveEndHour(novo_horario);
            }
            else if (comandoReal == "set_grace_period") {
              unsigned long grace_ms = (unsigned long)parametro * 60 * 1000;
              setGracePeriod(grace_ms);
            }
          }
        }
      }
      // Passo 4: O NestJS enviou um Ping ("2") para saber se a balança está viva!
      else if (length > 0 && payload[0] == '2') 
      {
        // Se não respondermos com "3" rapidamente, o Render corta a internet.
        webSocket.sendTXT("3");
      }
      break;
    case WStype_PING:
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

  struct tm struct_last_sip_time;
  localtime_r(&last_sip_time, &struct_last_sip_time);

  Serial.println("Hydration data loaded successfully:");
  Serial.print("Daily Consumed: "); 
  Serial.print(daily_consumed); 
  Serial.println(" ml");
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
// void handleDayChange()
// {
//   struct tm time_now;

//   if (!getLocalTime(&time_now, 0)) // O relógio ainda não está sincronizado com NTP?
//   {
//     // Se não estiver, retorna sem fazer nada
//     return; 
//   }

//   bool apply_reset = false;

//   if (was_active && !is_active) // Estava ativo e não esta mais?
//   {
//     apply_reset = true;
//   }
//   else if (!is_active && (last_reset_day != time_now.tm_mday)) // Não esta ativo e ainda não resetou hoje? (Provavelmente ESP32 reiniciou fora do horario ativo)
//   {
//     apply_reset = true;
//   }

//   if (apply_reset)
//   {
//     Serial.println("===============================================================================");
//     Serial.println("New date detected! Reseting consumption...");

//     current_state = IDLE;
//     deficit = 0;

//     daily_consumed = 0;
//     last_reset_day = time_now.tm_mday; // Atualiza o ultimo reset para o dia atual

//     Preferences preferences;
//     preferences.begin("hydration", false);
//     preferences.putInt("daily_consumed", 0);
//     preferences.putInt("last_reset_day", last_reset_day);
//     preferences.end();

//     Serial.println("===============================================================================");
//   }
  
//   was_active = is_active;
// }
void handleDayChange()
{
  struct tm time_now;

  if (!getLocalTime(&time_now, 0)) // O relógio ainda não está sincronizado com NTP?
  {
    // Se não estiver, retorna sem fazer nada
    return; 
  }

  bool apply_reset = false;

  if (was_active && !is_active) // Estava ativo e não esta mais? (Encerrou o expediente)
  {
    apply_reset = true;
  }
  else if (last_reset_day != time_now.tm_mday) // É um novo dia no calendário e ainda não resetou hoje?
  {
    int current_mins = time_now.tm_hour * 60 + time_now.tm_min;
    int start_mins = active_start_hour.hour * 60 + active_start_hour.minute;
    int end_mins = active_end_hour.hour * 60 + active_end_hour.minute;
    
    bool crosses_midnight = (start_mins > end_mins); // O horário ativo passa da meia-noite?
    
    if (crosses_midnight && current_mins <= end_mins) // A janela passa da meia-noite e ainda estamos na madrugada? (O expediente lógico de hoje ainda não acabou)
    {
       // Não faz nada (O reset vai acontecer naturalmente quando o expediente atual acabar)
    } 
    else 
    {
       apply_reset = true; // Estamos de fato em um novo dia. Confirma o reset
    }
  }

  if (apply_reset) // A flag de reset foi acionada?
  {
    Serial.println("===============================================================================");
    Serial.println("New date or active period detected! Reseting consumption...");

    current_state = IDLE;
    deficit = 0;

    daily_consumed = 0;
    last_reset_day = time_now.tm_mday; // Atualiza o ultimo reset para o dia atual

    Preferences preferences;
    preferences.begin("hydration", false);
    preferences.putInt("daily_consumed", 0);
    preferences.putInt("last_reset_day", last_reset_day);
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

  // // OBRIGATÓRIO PARA O SOCKET.IO v4: O ESP32 tem de enviar Ping ("2") a cada ~20s
  // if (millis() - last_ws_ping > 15000) {
  //   if (webSocket.isConnected()) {
  //       webSocket.sendTXT("2");
  //   }
  //   last_ws_ping = millis();
  // }

  // if (millis() - last_heap_print > 5000) {
  //   Serial.printf("[MEM] Heap livre: %d bytes\n", ESP.getFreeHeap());
  //   last_heap_print = millis();
  // } 

  if (millis() - last_http_ping > 120000) { // A cada 2 minutos
  last_http_ping = millis();
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    char url[128];
    snprintf(url, sizeof(url), "https://%s/dispositivos/comando/%s", WS_HOST, TOKEN_ACESSO);
    http.begin(url);
    http.GET();
    http.end();
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
    

  //   // Horario atual
  //   Serial.print("time:");
  //   Serial.printf("%02d:%02d:%02d", struct_real_time.tm_hour, struct_real_time.tm_min, struct_real_time.tm_sec);
  //   Serial.print(", ");

  //   // Horário acelerado
  //   Serial.print("accelerated_time:");
  //   Serial.printf("%02d:%02d:%02d", struct_fsm_time.tm_hour, struct_fsm_time.tm_min, struct_fsm_time.tm_sec); 
  //   Serial.print(", ");
  
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

  //   // Horario do ultimo alerta
  //   Serial.print("last_alert_time:");
  //   if (last_alert_time == 0) Serial.print("00:00:00");
  //   else Serial.printf("%02d:%02d:%02d", struct_last_alert_time.tm_hour, struct_last_alert_time.tm_min, struct_last_alert_time.tm_sec); 
  //   Serial.print(", ");

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
    Serial.print("sinal_filtrado:");
    Serial.println(filtered_weight, 2);    
  }
}

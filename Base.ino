#include "HX711.h"             // Biblioteca para comunicação com o conversor A/D da balança
#include <Preferences.h>       // Biblioteca padrão para leitura/escrita na memória flash (NVS)
#include <WiFi.h>              // Biblioteca padrão para conexão WiFi
#include <WiFiManager.h>       // Biblioteca para gerenciamento de rede (Portal Cativo)
#include "time.h"              // Biblioteca padrão do C++ para lidar com relógio e data

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
const Hour ACTIVE_START_HOUR = {21, 40}; // Horário de inicio das atividades da balança
const Hour ACTIVE_END_HOUR = {0, 59};// Horário de fim das atividades da balança
const unsigned long GRACE_PERIOD = 900000; // Periodo de carência (15 minutos em ms)

// Configurações de hidratação
const int DAILY_GOAL = 2000; // Meta diaria
/*==================================================================================*/

/* --- Constantes e parâmetros --- */
/*==================================================================================*/
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
/*==================================================================================*/

/* --- Variáveis globais e instâncias --- */
/*==================================================================================*/
HX711 scale; // Instância do HX711

// Vetores de armazenamento para os filtros
float samples_median[NUM_SAMPLES_MEDIAN] = {0.0};
float sample_ema = 0.0;

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

          daily_consumed += consumed_volume; 
          last_sip_time = time(NULL); 

          Preferences preferences;
          preferences.begin("hydration", false);
          preferences.putInt("daily_consumed", daily_consumed);
          preferences.putLong64("last_sip_time", (int64_t)last_sip_time);
          preferences.end();

          Serial.print("Consumed water: ");
          Serial.print(consumed_volume);
          Serial.println(" ml");
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

  Preferences preferences;
  preferences.begin("config", false); 

  unsigned long saved_grace_period = preferences.getULong("grace_period", 0);

  if (saved_grace_period == 0) 
  {
    Serial.println("Configs not found in NVS. Applying default configurations...");

    preferences.putBytes("active_start_hour", &ACTIVE_START_HOUR, sizeof(Hour));
    preferences.putBytes("active_end_hour", &ACTIVE_END_HOUR, sizeof(Hour));
    
    preferences.putULong("grace_period", GRACE_PERIOD);
    preferences.putInt("daily_goal", DAILY_GOAL);

    active_start_hour = ACTIVE_START_HOUR;
    active_end_hour = ACTIVE_END_HOUR;
    grace_period = GRACE_PERIOD;
    daily_goal = DAILY_GOAL;
  }
  else
  {
    Serial.println("Configs found in NVS successfully!");

    preferences.getBytes("active_start_hour", &active_start_hour, sizeof(Hour));
    preferences.getBytes("active_end_hour", &active_end_hour, sizeof(Hour));
    
    grace_period = saved_grace_period; 
    daily_goal = preferences.getInt("daily_goal", DAILY_GOAL);
  }

  preferences.end(); 

  Serial.println("System configurations applied successfully:");
  Serial.printf("Start Time: %02d:%02d\n", active_start_hour.hour, active_start_hour.minute);
  Serial.printf("End Time: %02d:%02d\n", active_end_hour.hour, active_end_hour.minute);
  Serial.print("Grace Period (ms): "); Serial.println(grace_period);
  Serial.print("Daily Goal (mL): "); Serial.println(daily_goal);
  Serial.println("===============================================================================");
  
  delay(1000);
}

void setupHydration()
{
  Serial.println("===============================================================================");
  Serial.println("Loading hydration data from NVS...");

  Preferences preferences; 
  preferences.begin("hydration", false); 

  daily_consumed = preferences.getInt("daily_consumed", 0);  
  last_sip_time = (time_t)preferences.getLong64("last_sip_time", 0);
  last_reset_day = (time_t)preferences.getInt("last_reset_day", 0);

  preferences.end(); 

  Serial.println("Hydration data loaded successfully:");
  Serial.print("Daily Consumed: "); 
  Serial.print(daily_consumed); 
  Serial.println(" ml");
  Serial.print("Last sip time: "); 
  Serial.println(last_sip_time); 
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

/* --- Motor de animações visuais e sonoras --- */
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
/*==================================================================================*/


/* --- Controle de Tempo e Datas --- */
/*==================================================================================*/
void handleDayChange()
{
  struct tm time_now;

  if (!getLocalTime(&time_now, 0)) // O relógio já está sincronizado com NTP?
  {
    // Se não estiver, retorna sem fazer nada
    return; 
  }

  bool apply_reset = false;

  if (was_active && !is_active) // Estava ativo não esta mais?
  {
    apply_reset = true;
  }
  else if (!is_active && (last_reset_day != time_now.tm_mday)) // Não esta ativo e ainda não resetou hoje? (Provavelmente ESP32 reiniciou fora do horario ativo)
  {
    apply_reset = true;
  }

  if (apply_reset)
  {
    Serial.println("===============================================================================");
    Serial.println("New date detected! Reseting consumption...");

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
// void handleDayChange()
// {
//   if (last_sip_time == 0) // O horario do ultimo gole esta zerado?
//   {
//     return;
//   }

//   struct tm tm_now;
//   localtime_r(&now, &tm_now);
  
//   struct tm tm_last;
//   localtime_r(&last_sip_time, &tm_last);

//   if (tm_now.tm_mday != tm_last.tm_mday || tm_now.tm_mon != tm_last.tm_mon) // O dia ou mês mudaram?
//   {
//     Serial.println("===============================================================================");
//     Serial.println("New date detected! Reseting consumption...");

//     current_state = IDLE;
//     deficit = 0;

//     daily_consumed = 0;

//     Preferences preferences;
//     preferences.begin("hydration", false);
//     preferences.putInt("daily_consumed", 0);
//     preferences.end();

//     Serial.println("===============================================================================");

//   }
// }

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
// void handleSerialCommands() 
// {
//   if (Serial.available() > 0) 
//   {
//     char cmd = Serial.read(); 
    
//     if (cmd == 'G' || cmd == 'g') {
//       Serial.println("\n[TESTE] >>> Disparando Alerta VERDE! <<<");
//       triggerAlert(GREEN, true, true); 
//     }
//     else if (cmd == 'Y' || cmd == 'y') {
//       Serial.println("\n[TESTE] >>> Disparando Alerta AMARELO! <<<");
//       triggerAlert(YELLOW, true, true);
//     }
//     else if (cmd == 'R' || cmd == 'r') {
//       Serial.println("\n[TESTE] >>> Disparando Alerta VERMELHO! <<<");
//       triggerAlert(RED, true, true);
//     }
//     else if (cmd == 'P' || cmd == 'p') {
//       Serial.println("\n[TESTE] >>> Forçando a parada das animações! <<<");
//       playing_alert = IDLE;
//       noTone(BUZZER);
//       turnOffAllLeds();
//     }
//   }
// }

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
}

void loop() 
{
  now = time(NULL); // Atualiza o horario (Tem que fazer isso antes de qualquer outra coisa)

  // handleSerialCommands(); // Gerencia comandos recebidos pela porta Serial
  handleWifiConnection(); // Gerencia Conexão WiFi
  handleHardReset(); // Gerencia botão "en" para restaurar dados de fabrica
  
  // Temporario
  switch (current_state)
  {
    case IDLE:
      turnOffAllLeds();
      break;
    case GREEN:
      turnOnLed(0, LOW, HIGH, LOW); 
      turnOnLed(1, LOW, HIGH, LOW); 
      turnOnLed(2, LOW, HIGH, LOW); 
      turnOnLed(3, LOW, HIGH, LOW); 
      turnOnLed(4, LOW, HIGH, LOW); 
      break;
    case YELLOW:
      turnOnLed(0, HIGH, HIGH, LOW); 
      turnOnLed(1, HIGH, HIGH, LOW); 
      turnOnLed(2, HIGH, HIGH, LOW); 
      turnOnLed(3, HIGH, HIGH, LOW); 
      turnOnLed(4, HIGH, HIGH, LOW); 
      break;
    case RED:
      turnOnLed(0, HIGH, LOW, LOW); 
      turnOnLed(1, HIGH, LOW, LOW); 
      turnOnLed(2, HIGH, LOW, LOW); 
      turnOnLed(3, HIGH, LOW, LOW); 
      turnOnLed(4, HIGH, LOW, LOW); 
      break;
  }

  if(scale.is_ready())
  {
    float weight = scale.get_units(1); 
    
    float median_weight = median(weight, samples_median, NUM_SAMPLES_MEDIAN);
    float filtered_weight = exponentialMovingAverage(median_weight, NUM_SAMPLES_MEDIAN); 

    current_consumed_volume = getConsumption((int)filtered_weight); // Calcula o volume consumido (ou não)

    struct tm time_now; 
    localtime_r(&now, &time_now);

    struct tm struct_last_sip_time; 
    localtime_r(&last_sip_time, &struct_last_sip_time);

    struct tm struct_last_alert_time; 
    localtime_r(&last_alert_time, &struct_last_alert_time);

    // Printa os dados da maquina de estados
    printFiniteStateMachineData();
    
    // // Instante exato desde o inicio do funcionamento do ESP32
    // Serial.print("instant:");
    // Serial.print(millis()); 
    // Serial.print(", ");

    // // Horario atual
    // Serial.print("time:");
    // Serial.printf("%02d:%02d:%02d", time_now.tm_hour, time_now.tm_min, time_now.tm_sec);
    // Serial.print(", ");

    // // Horario do ultimo gole
    // Serial.print("last_sip_time:");
    // if (last_sip_time == 0) Serial.print("00:00:00");
    // else Serial.printf("%02d:%02d:%02d", struct_last_sip_time.tm_hour, struct_last_sip_time.tm_min, struct_last_sip_time.tm_sec); 
    // Serial.print(", ");

    // // Horario do ultimo alerta
    // Serial.print("last_alert_time:");
    // if (last_alert_time == 0) Serial.print("00:00:00");
    // else Serial.printf("%02d:%02d:%02d", struct_last_alert_time.tm_hour, struct_last_alert_time.tm_min, struct_last_alert_time.tm_sec); 
    // Serial.print(", ");

    // Peso bruto
    Serial.print("weight:");
    Serial.print(weight, 2); 
    Serial.print(", ");

    // Peso filtrado
    Serial.print("sinal_filtrado:");
    Serial.println(filtered_weight, 2); 

    handleFiniteStateMachine(current_consumed_volume); // Gerencia os estados da maquina de estados (Tem que ser no fim do Loop por causa do current_consumed_volume)
    handleDayChange(); // Gerencia mudança de datas (Tem que ser no fim do Loop porque precisa do valor atualizado de is_active)
  }
}

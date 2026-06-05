# SHI — Sistema de Hidratação Inteligente (Firmware)

Projeto de Trabalho de Conclusão de Curso (TCC) de Engenharia da Computação desenvolvido por **Fabricio** e **Gabriel**.

Este repositório contém o firmware em C++ para o microcontrolador ESP32. Ele é responsável pela medição física do consumo de água, processamento de sinais, feedback visual/sonoro em tempo real e comunicação bidirecional via WebSocket com a [API NestJS do projeto](https://github.com/gabrielorod/tcc-shi-api).

## 🛠 Stack Tecnológica

- **Hardware:** Microcontrolador ESP32 + Módulo HX711 (Conversor A/D) + Célula de Carga
- **Framework:** Arduino Core para ESP32 (C++)
- **Conectividade:** WiFiManager (Portal Cativo para redes)
- **Tempo real:** WebSocketsClient (integração direta com Socket.io v4)
- **Armazenamento:** Partição NVS (Preferences) para Buffer Offline e Configurações
- **Processamento de Sinais:** Filtro EMA (Média Móvel Exponencial) + Filtro de Mediana
- **Serialização:** ArduinoJson

---

## 🔌 Esquema de Pinagem (Pinout)

O hardware foi projetado e montado utilizando os seguintes pinos do ESP32:

### Sensores e Controle
| Componente | Pino ESP32 | Descrição |
| :--- | :--- | :--- |
| **HX711 (DT)** | `GPIO 34` | Pino de dados da célula de carga |
| **HX711 (SCK)** | `GPIO 2` | Pino de clock da célula de carga |
| **Buzzer** | `GPIO 15` | Buzzer passivo para alertas e melodias |
| **Botão Reset** | `GPIO 0` | Botão BOOT nativo para Hard Reset (Restaurar Padrões) |

### LEDs RGB (Feedback Visual)
| LED | Vermelho (R) | Verde (G) | Azul (B) |
| :---: | :---: | :---: | :---: |
| **LED 0** | `GPIO 27` | `GPIO 14` | `GPIO 32` |
| **LED 1** | `GPIO 33` | `GPIO 25` | `GPIO 26` |
| **LED 2** (Centro) | `GPIO 23` | `GPIO 22` | `GPIO 21` |
| **LED 3** | `GPIO 17` | `GPIO 16` | `GPIO 4` |
| **LED 4** | `GPIO 13` | `GPIO 19` | `GPIO 18` |

---

## 📦 Dependências e Bibliotecas

Para compilar este projeto na Arduino IDE ou PlatformIO, instale as seguintes bibliotecas através do **Gerenciador de Bibliotecas**:

- `HX711 Arduino Library` (por Bogdan Necula)
- `WiFiManager` (por tzapu)
- `WebSockets` (por Markus Sattler)
- `ArduinoJson` (por Benoit Blanchon)

---

## 🚀 Instalação e Uso

1. Compile e faça o upload do código para o ESP32.
2. Na primeira inicialização (ou caso a rede caia), o ESP32 criará um Access Point chamado **`Balanca SHI`**.
3. Conecte-se a essa rede Wi-Fi usando a senha: **`balanca_shi`**.
4. O Portal Cativo abrirá automaticamente. Selecione a rede Wi-Fi do local e insira a senha.
5. O ESP32 reiniciará, conectará à internet, sincronizará o relógio mundial (NTP) e estabelecerá o túnel WebSocket com o Backend.

---

## 🧠 Fluxo Arquitetural e Funcionalidades

- **Máquina de Estados (FSM):** Avalia o déficit de hidratação cruzando a meta diária com o horário atual de expediente do usuário. Controla automaticamente os alertas `GREEN`, `YELLOW` e `RED`.
- **Buffer Offline (Store & Forward):** Se a conexão com a internet cair, a balança continua a calcular os goles e guarda-os numa "mochila" na memória Flash (NVS). Quando a conexão é restaurada, os dados pendentes são injetados automaticamente no servidor.
- **Detecção Avançada de Goles:** Algoritmo que detecta retirada do copo, reabastecimento fantasma e goles contínuos (com canudo) através da análise da variação do filtro matemático com tempo de acomodação (Settling Time) de 1.5s.
- **Telemetria de 1Hz:** A balança emite o peso filtrado em cima da base 1 vez por segundo (1Hz), permitindo que o aplicativo Front-end mostre a variação em tempo real.
- **Calibração Segura:** Rotina de calibração assíncrona (não-bloqueante) que permite o cancelamento remoto pelo App sem corromper as configurações de `offset` e `scale_divider` anteriores.

---

## 📡 Eventos WebSocket (Socket.io)

O firmware utiliza comunicação bidirecional 100% via WebSocket para máxima eficiência.

### 📤 Emitidos pela Balança (ESP32 → API)
| Evento | Payload JSON | Descrição |
| :--- | :--- | :--- |
| `registrar_leitura` | `{tokenAcesso, quantidadeMl}` | Envia o volume de água bebido. |
| `registrar_calibracao`| `{tokenAcesso, pesoVazioG, recipienteId}` | Finaliza a Tara de um recipiente. |
| `peso_em_tempo_real` | `{tokenAcesso, pesoAtual}` | Telemetria contínua do peso em cima da mesa. |
| `ping_dispositivo` | `{tokenAcesso}` | Heartbeat enviado a cada 30 segundos (Keep-Alive lógico). |
| `calibracao_status` | `{tokenAcesso, mensagem}` | Feedback visual para o App durante a calibração física. |

### 📥 Escutados pela Balança (API → ESP32)
Esses eventos são recebidos encapsulados na chave `"comando"`.

| Comando | Ação no ESP32 |
| :--- | :--- |
| `set_container_weight` | Sincroniza o peso base do recipiente ativo para descontar da leitura. |
| `set_daily_consumed` | Sincroniza a Máquina de Estados com o que já foi bebido hoje. |
| `set_goal` | Atualiza a meta diária de hidratação na memória permanente. |
| `calibrate` | Inicia rotina interativa de calibração para achar o divisor de escala. |
| `cancel_calibration` | Aborta a rotina de calibração física, restaurando a balança. |
| `commit_calibration` | Confirma e salva a calibração física recém-feita na NVS. |
| `alerta_hidratacao` | Força a reprodução do alerta sonoro e visual de hidratação (Amarelo). |
| `restore` | Formata a memória NVS, apaga credenciais de Wi-Fi e reinicia o chip. |

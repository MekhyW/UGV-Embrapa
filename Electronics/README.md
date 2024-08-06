# Eletrônica

## Esquemáticos e Detalhamento Geral de Funcionamento

A eletrônica do protótipo é composta por duas placas: a placa de acionamento dos motores (Motor Board), onde estão localizados os circuitos de potência e a placa responsável por comandar todo o funcionamento do sistema (Brain Board), onde estão circuitos de controle e está alocada uma Raspberry Pi 4, modelo B de 8 GB, com microprocessador quad-core de 64-bits ARM-Cortex A72, rodando a 1,5 GHz.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/1dc84aa9-ae4c-47ea-b9fe-22ce198eccc0)

### Diagramas de Alimentação 

Analisando primeiro a alimentação do sistema, uma bateria fornecendo 14,8V DC é conectada diretamente à placa de acionamento, passando por um circuito de proteção, que possui um fusível, um resistor de 1 kΩ e um diodo, que atuam como proteção do circuito dos motores. Além disso, um módulo de monitoramento de tensão e corrente (INA260) é utilizado para medição de sobrecorrente, além de um multímetro, que é utilizado para monitoramento do usuário dos dados de corrente e tensão.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/9f748100-b6f7-4ea9-af1d-d4a25ad7c739)


A tensão 14,8V é usada na alimentação de dois reguladores, um step-down de 12V (D24V22Fx), usado para alimentar os drivers da Roboclaw e outro de 5V (D24V150Fx). Há também um terceiro regulador de tensão que converte a tensão de 5V para 3,3V (MIC2937A-3.3WT), alimentando o módulo de expansão de PWM (PCA9685), o monitor de tensão e corrente e os LED’s de sinalização de ambas as placas. A tensão de 5V, por sua vez, também realiza a alimentação do módulo de expansão de PWM, dos servos, da Raspberry Pi, do display da IHM e dos encoders dos motores. Os motores são alimentados pela Roboclaw via PWM com tensão máxima de 12V.

### Diagramas de Sinais

Para o diagrama de sinais do sistema, note que um novo bloco foi adicionado, o conjunto entre o operador e o rádio controle, responsável por enviar os comandos por meio de ondas de rádio até a Raspberry Pi, que recebe o sinal por meio de um módulo USB. Os dados recebidos pelo microprocessador são enviados na serial e usados para o controle dos motores e servos. 

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/25b216b0-da69-4218-9a60-26376cbdb3fe)

A malha de controle dos seis motores é comandada pela Raspberry Pi, que recebe a referência do rádio controle e envia os comandos para os drivers da Roboclaw, que realizam o acionamento via PWM. O fechamento da malha é realizado pelos encoders, que são responsáveis pelo sensoriamento, enviando os dados coletados ao controlador presente no driver.

Os quatro servos, por sua vez, são controlados em malha aberta (fechada internamente) acionados por meio de um sinal PWM. Este sinal é fornecido pelo módulo PCA9685 (expansão), usado para ampliar a quantidade de servos que podem ser controlados, por meio da geração de 16 novos canais PWM. A comunicação desse módulo com a Raspberry Pi ocorre por meio do protocolo de comunicação I2C.

Considerando a comunicação do micro-computador com o módulo de monitoramento de tensão e corrente (INA260), nota-se que o envio de dados também ocorre por meio de um protocolo de comunicação I2C. O módulo também se comunica com os LED’s de sinalização para alertar o usuário caso seu resistor de shunt interno detecte alguma alteração no sistema de alimentação.

Além do módulo de expansão e do monitor, a comunicação com o display da IHM, que será futuramente implementada, também ocorrerá de acordo com o protocolo I2C. O motivo para tal escolha é que a placa de controle apresenta um barramento I2C disponível, que poderia ser adaptado para a conexão do display com baixa quantidade de adaptações necessárias.

Outra observação importante em relação ao diagrama é que a Raspberry Pi também está responsável pelo controle do regulador de tensão de 12V por meio de um sinal de enable, ligando ou desligando o output de 12V.

Analisando um pouco mais a placa de controle, observa-se a presença de alguns LED’s de sinalização, esses componentes possuem como finalidade a comunicação com o usuário, sinalizando por exemplo quando ocorre a transferência de dados pela serial, acendendo quando os bits são transferidos. 

Por fim, o último fluxo de dados a ser analisado é o indicador do estado de emergência, no qual um sinal digital de nível lógico alto é enviado pelo botão de emergência ao microprocessador, que comunica a situação ao usuário por meio do display da IHM. 

## Circuito de Emergência Físico

O esquemático do circuito de emergência físico utilizado para desernegizar o robô está ilustrado a seguir.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/a42c251f-cbff-423a-822a-de9d12807db7)

Para indicação do estado de emergência ao usuário, foi adotado um sinaleiro vermelho, que acende assim que o botão for pressionado. O componente apresenta uma resistência interna de 1800 Ω, suportando tensões de até 24V e seu acionamento é realizado por uma chave normalmente aberta, responsável por fechar o circuito após pressionamento do botão. 

Foi implementado também um botão de emergência remoto, cujo funcionamento está explicado na pasta de programação. 

## Detalhamento Circuito Display

O esquemático de conexão de pinos do display para garantir a comunicação I2C está representado a seguir. Os pinos para conexão de 5V e GND estão disponíveis na raspberry pi e uma extensão dos pinos SCL e SDA pode ser encontrada na placa de controle.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/89fe5d82-cf03-44a6-9ce3-b9f463eb3dc9)

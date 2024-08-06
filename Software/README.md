# Software do UGV para Monitoramento de Talhões em Fruticultura e Silvicultura

## Visão Geral

Este projeto visa desenvolver um Veículo Terrestre Não Tripulado (UGV) para monitoramento de talhões em fruticultura e silvicultura, inspirado no projeto Open Source Rover da NASA. A seguir, detalhamos a configuração do software utilizado no UGV e os resultados obtidos com o sistema.

## Preparação do Ambiente de Desenvolvimento

### Instalação do Sistema Operacional

1. **Baixar a Imagem do Ubuntu Server**:
   - Acesse o [site oficial do Ubuntu Server](https://ubuntu.com/download/server) e baixe a imagem mais recente.
   - Utilize um software como o Raspberry Pi Imager para gravar a imagem no cartão SD.

2. **Configuração Inicial**:
   - Insira o cartão SD na Raspberry Pi e ligue o dispositivo.
   - Configure a rede Wi-Fi e outras preferências básicas durante a inicialização.
   - Atualize o sistema operacional:
     ```sh
     sudo apt update
     sudo apt upgrade
     ```

### Configuração da Raspberry Pi como Servidor e Comunicação SSH

1. **Modo Servidor**:
   - A configuração do servidor permite que a Raspberry Pi seja acessada remotamente.
   - Conecte-se ao servidor utilizando SSH:
     ```sh
     ssh embrapa@10.42.0.1
     ```

## Funcionamento do Controle

O controle remoto do UGV utiliza um transmissor de rádio controle Spektrum DXS, que comunica com um receptor instalado no UGV. O mapeamento dos botões e alavancas do controle foi configurado para facilitar a operação do UGV, permitindo controle de velocidade, direção, rotação e acionamento de funções específicas. Para configurar o controle remoto, mapeamos eixos e botões utilizando o nó `joy` do ROS 2.

### Parâmetros do Controle

- **scale_linear.x**: 0.12
- **axis_linear.x**: 1
- **axis_angular.yaw**: 2
- **axis_angular.pitch**: 4
- **scale_angular.yaw**: 1.25
- **scale_angular.pitch**: 0.25
- **scale_angular_turbo.yaw**: 0.0
- **scale_linear_turbo.x**: 0.0
- **enable_button**: 0
- **enable_turbo_button**: 1

### Rotina de Emergência

Foi implementada uma rotina de emergência para garantir a segurança operacional do rover. Ao acionar o botão de emergência, o valor de `scale_linear_turbo` é ajustado para 0.0, resultando na parada completa do rover. Foi feito um fluxograma mostrando essa rotina.

_Fluxograma de funcionamento do UGV_

![Fluxograma de emergencia](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/49559187/f8c5dcd0-ad6a-4c44-ac0e-e2f40bb0a978)


# Mapeamento dos Botões e Alavancas

1. Velocidade:

   A alavanca esquerda controla a velocidade do UGV. Movendo a alavanca para frente, o UGV avança; movendo-a para trás, o UGV recua. Esta configuração é intuitiva 
   e permite um controle preciso da velocidade.

2. Direção:

   A alavanca direita é responsável pela direção do UGV. Movendo a alavanca para a esquerda ou direita, o UGV gira na respectiva direção. Este controle é 
   essencial para navegação em terrenos variados.

3. Liga/Desliga:

   O botão central inferior permite ligar ou desligar o UGV. Esta função é crucial para iniciar ou interromper a operação do UGV rapidamente.

4. Emergência:

   O botão de emergência, localizado na parte superior esquerda do controle, pode ser acionado para parar imediatamente o UGV. Este botão é mapeado para ajustar o    valor de scale_linear_turbo para 0.0, resultando na parada completa do veículo em situações de emergência.

5. Rotação:

   A rotação do UGV é controlada por um interruptor na parte superior direita do controle. Esta funcionalidade é utilizada para ajustes finos de rotação,       
   permitindo manobras precisas.

6. Deadman:

   O botão Deadman, localizado na parte traseira do controle, deve ser pressionado continuamente para manter o UGV operando. Soltar este botão resulta na parada 
   imediata do UGV, como uma medida de segurança adicional.

# Operação
Para operar o UGV, o usuário deve:

1. Ligar o Controle e o UGV:

   Pressione o botão de liga/desliga no controle e, em seguida, ligue o UGV.

2. Ajustar a Velocidade e Direção:

   Utilize as alavancas para controlar a velocidade e direção do UGV conforme necessário para a missão.

3. Monitorar a Operação:

   Esteja atento à resposta do UGV e ajuste os controles conforme necessário. Use a alavanca de rotação e o botão de emergência para manobras e paradas rápidas.

4. Segurança:

   Mantenha sempre um dedo no botão Deadman para garantir a operação segura. Em caso de emergência, utilize o botão de emergência para parar o UGV imediatamente.

# Foto do controle

![Controle frente](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/49559187/aa9ca24c-7300-449e-a2a1-b97dc7d0b22e)

![Controle atrás](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/49559187/849eaf65-5603-4b8a-b43f-8160a4e89344)


### Passos para Instalação

1. **Atualização do Sistema:**
   - `sudo apt update`
   - `sudo apt upgrade`

2. **Instalação do ROS 2 Humble:**
   - Seguir as instruções oficiais para instalação do ROS 2 Humble.

### Bringup Manual do Rover

Para iniciar manualmente o rover, execute os seguintes comandos no terminal:

1. Certificar que o programa já não esteja sendo executado:
   ```bash
   sudo systemctl stop osr_startup
   ```

2. Executar o comando de bringup:
   ```bash
   ros2 launch osr_bringup osr_launch.py
   ```

### Bringup Automático com Script de Lançamento

Para automatizar o bringup e eliminar a necessidade de acesso via SSH, configuramos a Raspberry Pi para iniciar o código do rover automaticamente.

1. Navegar até a pasta `init_scripts`:
   ```bash
   cd ~/osr_ws/src/osr-rover-code/init_scripts
   ```

2. Criar links simbólicos:
   ```bash
   sudo ln -s $(pwd)/launch_osr.sh /usr/local/bin/launch_osr.sh
   sudo ln -s $(pwd)/osr_paths.sh /usr/local/bin/osr_paths.sh
   ```

3. Copiar o arquivo de serviço para o diretório de serviços do systemd:
   ```bash
   sudo cp osr_startup.service /etc/systemd/system/osr_startup.service
   ```

4. Ajustar as permissões do arquivo de serviço:
   ```bash
   sudo chmod 644 /etc/systemd/system/osr_startup.service
   ```

### Tabela de Comandos do Bringup Automático

| Comando                          | Descrição                           |
|----------------------------------|-------------------------------------|
| `sudo systemctl start osr_startup` | Inicia o serviço manualmente       |
| `sudo systemctl stop osr_startup`  | Para o serviço manualmente          |
| `sudo systemctl enable osr_startup`| Habilita o serviço no boot         |
| `sudo systemctl disable osr_startup`| Desabilita o serviço no boot       |
| `sudo systemctl status osr_startup`| Verifica o status do serviço       |


### Configuração display LCD 16x02 com Módulo de Conversão I²C

É necessário alimentá-lo com 5V em um dos pinos de alimentação disponíveis e assinalados pelo silkscreen da brain board, bem como conectar os pinos SDA e SCL.

![Ligações do display](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/72100554/505222e8-aac1-4e31-95c6-8f10a74234f2)  

Após conectá-lo à placa, acessou-se o servidor via SSH e instalou-se a biblioteca `RPi_GPIO_i2c_LCD` pelo terminal:

```bash
sudo pip3 install RPi-GPIO-I2C-LCD
```

Com a biblioteca instalada, foi necessário buscar o endereço, (0x27), i2c do display usando o comando:
```bash
ubuntu@embrapa:~$ i2cdetect -y 1
```
A estratégia adotada para redução de consumo foi atualizar o display em um determinado tempo, para isso foi necessário adicionar via editor ‘nano’ um comando para executar o script a cada 5 minutos:
```bash
ubuntu@embrapa:~$ crontab -e
```

Na última linha adicionou-se o código:
```bash
*/5 * * * * /usr/bin/python3 /home/ubuntu/osr_ws/src/ihm_lcd.py
```

### Aquisição de Imagens

1. **Instalação de Bibliotecas Necessárias**:
   ```sh
   sudo apt install ros-humble-cv-bridge
   sudo apt install ros-humble-image-transport
   ```

2. **Configuração de Câmeras**:
   - Conecte as câmeras Intel RealSense D435i e Logitech C920 HD PRO na Raspberry Pi.
   - Utilize os seguintes comandos para verificar e ajustar as configurações:
     ```sh
     rs-enumerate-devices
     v4l2-ctl --list-devices
     ```

3. **Publicação e Visualização de Imagens**:
   - Execute os nós ROS para capturar e publicar imagens:
     ```sh
     ros2 run realsense2_camera realsense2_camera_node
     ros2 run image_transport republish raw in:=/camera/color/image_raw out:=/image_raw
     ```
   - Outra forma de inicializar a captação de imagens consiste em rodar o codigo abaixo:
     ```sh
     systemctl restart start_turtle.service
     ```
   - Para visualizar as imagens usar Rviz2 ou rqt_image_view:
   - rqt_image_view:
     ```sh
     ros2 run rqt_image_view rqt_image_view
     ```
   - Rviz2:
     ```sh
     Rviz2
     ```

### Instalação do ROS Humble

1. **Adicionar Chave do Repositório ROS**:
   ```sh
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update
   sudo apt install curl
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   ```

2. **Adicionar Repositório ROS**:
   ```sh
   sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

3. **Configurar Ambiente ROS**:
   Adicione as seguintes linhas ao arquivo `~/.bashrc`:
   ```sh
   source /opt/ros/humble/setup.bash
   ```

## Resultados

### Visualização e Monitoramento

1. **Interface Gráfica (Rviz2)**:
   - Utilizada para visualização dos tópicos ROS, incluindo as imagens capturadas pelas câmeras.
   - Permite monitorar a posição e o status dos sensores em tempo real.

2. **Aquisição de Imagens**:
   - As câmeras Intel RealSense D435i capturam imagens com profundidade, permitindo a criação de mapas 3D do ambiente.
   - As imagens são publicadas em tópicos ROS e podem ser visualizadas remotamente.

### Desempenho e Processamento

1. **Processamento em Tempo Real**:
   - A configuração em modo servidor da Raspberry Pi, aliada ao ROS, permite o processamento eficiente das imagens capturadas.
   - O sistema foi capaz de lidar com o processamento de múltiplos streams de imagem simultaneamente sem atraso significativo.

2. **Estabilidade e Confiabilidade**:
   - O sistema demonstrou estabilidade durante os testes, com a comunicação SSH permitindo controle remoto seguro e eficaz.
   - A integração com o ROS possibilitou a fácil adição e modificação de nós conforme necessário, garantindo flexibilidade para futuras expansões.

## Contribuidores

- Breno Alencar Araújo
- Bruno Morales Balkins
- Fernando Bichuette Assumpção
- Giulia Carolina Martins de Sampaio

Orientador: Prof. Dr. Vinícius Licks
Mentor: Thiago Teixeira Santos



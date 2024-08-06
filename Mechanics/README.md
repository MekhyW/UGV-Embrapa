# Mecânica

## Modelo 3D
O modelo base 3D pode ser encontrado em [OnShape](https://cad.onshape.com/documents/e4f00b1a3d2edb1a84bbba1c/w/8ab8f394324bcc586236ef5d/e/9191e5ad2a70f387b419bc55?renderMode=0&uiState=645ede92f3a1a9205158b296) ou no arquivo STEP fornecido acima.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/72100554/a2137652-97fa-4312-889d-916f1c728e59)


## Adaptações da Documentação Oficial
Foram necessárias algumas adaptações na montagem da documentação oficial, uma vez que os componentes eram importados e parte deles não estava disponível para entrega. Para contornar esse problema, foram compradas peças equivalentes que permitissem adaptação para a aplicação proposta.
A seguir, as peças que passaram por esse processo adaptativo estão listadas e comentadas.
### Perfis da Suspensão
As primeiras peças a serem adaptadas foram os perfis de alumínio que compõem parte da suspensão do robô, uma vez que não foi possível encontrá-los nos comprimentos especificados na documentação oficial da Nasa, de 96mm. Como alternativa foram comprados perfis de tamanho superior, que foram cortados na dimensão correta.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/4b00c4d5-f164-4cf2-847e-b903f16ecaa5)

Após o corte dos perfis, a rosca para alocação do parafuso M4 utilizado na montagem precisou ser refeita para todos os quatro furos de cada uma das peças cortadas. A operação de rosqueamento foi realizada manualmente e composta por três etapas, utilizando machos de tamanhos diferentes até chegar no tamanho desejado da rosca. A peça não precisou ser furada novamente, uma vez que seus furos eram passantes, abrangendo toda a estrutura, independente do comprimento.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/6dc17718-f09a-4de4-a52c-d962fd01e204)

### Barras de Alumínio do Corpo 
As barras de alumínio utilizadas na montagem do corpo do robô também precisaram ser adaptadas, uma vez que também foram compradas em comprimento superior ao especificado e depois cortadas no tamanho correto, de 96 mm.

Após realização das operações de corte nas peças, foi necessário realizar a furação das barras, uma vez que o furo existente não era passante como no perfil. Além disso, também foi necessário realizar o rosqueamento em duas etapas, bem como ocorreu com a outra peça citada anteriormente.

Na operação de furação foram realizadas as seguintes etapas para garantir um melhor alinhamento do furo:
1.	pintura da face a ser furada com um marcador.
2.	Remoção da tinta para marcação do furo com auxílio de um marcador de altura vertical. Traçado realizado com dimensão equivalente à metade do lado da face, garantindo que as retas se cruzem no centro.
3.	Realização de uma marcação central para garantir que a broca não seja inserida de forma desalinhada, etapa realizada com um equipamento de punção de centro, uma morsa e um martelo.
4.	Furação com auxílio de uma fresadora manual de bancada, buscando alinhar a ponta da broca com a marcação realizada na etapa anterior. Avançar e recuar a broca durante o processo, garantindo escoamento do cavaco e dissipação do calor gerado.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/f95e6519-0d76-4535-a70d-1e3676e8d23a)

O processo de rosqueamento também foi necessário e foi refeito de forma identica ao procedimento descrito para os perfis. 

A peça com o rolamento para encaixe da suspensão também não estava disponível no modelo original da documentação da NASA, dessa forma foi adotada outra com diâmetro interno equivalente (Figura 28). Embora a substituta não encaixe perfeitamente e cause uma leve deformação estrutural, optou-se por não realizar nenhuma adaptação pois os componentes poderiam ser danificados e não possuíam substitutos. O efeito de deformação resultante não comprometeu o funcionamento do rover. 

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/749860f7-e1b6-4cc8-a596-2a6e98edfee1)

## Revisão de Erros da Documentação Oficial 

Um dos componentes da suspensão foi identificado erroneamente na documentação oficial, com código inicial 1601, que na realidade corresponde ao componente número 1611. Na visualização 3D da montagem, a peça identificada com código correspondente ao informado na documentação oficial apresenta diâmetro interno com valor inferior a 8mm, não montando no eixo especificado.

![image](https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/62897902/36c3ddc6-d7ef-491b-82d5-7051012ea459)

## Componentes Adicionais Fabricados

Para garantir a adaptabilidade do robô ao ambiente de monitoramento, alguns componentes adicionais precisaram ser fabricados, dentre eles os suporte das câmeras e o do LiDAR, permitindo que esses componentes sejam futuramente alocados na estrutura.

### Suporte das Câmeras

Como requisitado pelo cliente, também foram desenvolvidos alguns suportes para câmeras e um LIDAR, utilizando o software Fusion 360 para modelagem e a tecnologia de impressão 3D para materialização do projeto.

<img width="470" alt="suporte1" src="https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/38721933/3ee1b43e-3dae-476a-853d-57575b2d889a">

<img width="474" alt="Screenshot at Jun 10 11-16-50" src="https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/38721933/ca6b2074-cb1d-4ac4-968d-367030aa3d81">


### Suporte do LiDAR

<img width="460" alt="suporte3" src="https://github.com/pfeinsper/unmaned-ground-vehicle-2024.1/assets/38721933/a7b702c6-86ae-4b73-a62c-17ce572221a8">


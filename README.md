# Trabalho Pratico 1 - Planejamento de Movimento de Robos

Este trabalho consiste na implementação de quatro algoritmos:

1. Algoritmo tipo TangentBUG para a navegação de um robô com acionamento diferencial simulado no StageROS em um ambiente com obstáculos. O robô deve estar equipado  com um sensorLasere deve se mover entre duas posições quaisquer, escolhidas pelo usuário em tempo de execução,sem colidir com os obstáculos. Caso não haja caminho  entre as posições escolhidas, o robô deve informar isto ao usuário em um tempo finito.

2. Implementar um controlador que faça um robô com acionamento diferencial convergir e circular eternamente uma curva em forma de oito(Lemniscata de Bernoulli). Os parâmetros da curvadevem ser parâmetros de entrada do programa.

<div style="text-align:center"><img src="https://github.com/israelfi/tp_1/blob/master/media/Peek%202020-12-24%2001-16.gif" width="400" height="400" /></div>

3. Implementar uma estratégia simples de função de potencial (Potencial Atrativo + Potencial Repulsivo) para navegar o  robô entre  duas  posições  quaisquer  num  ambiente com obstáculos.

4. Implemente o A* considerando um ambiente discretizado na forma de grid e navegue o robô no StageROS(ou outro simulador)de acordo com o plano gerado.

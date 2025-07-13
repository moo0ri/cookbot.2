import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from roboticstoolbox import mstraj


# Parâmetros Físicos do Robô (editar conforme necessário)
M = np.diag([1, 1.0, 0.02])  # Inércia das juntas
C = np.diag([0.01, 0.1, 0.01])  # Atrito
G = np.zeros(3)                 # Gravidade (se necessário)

# Ganhos PID (editáveis facilmente)
Kp = np.array([100, 200, 150])  # Ganhos proporcionais
Kd = np.array([10, 20, 15])     # Ganhos derivativos
Ki = np.array([0, 0, 0])        # Ganhos integrais (caso queira usá-los)

#TRAJETÓRIA COOKBOT

from traj_cookbot import t_des, q_des, qd_des, qdd_des

# Usando os resultados de mstraj
t_span = t_des  # Usar os tempos gerados por mstraj
qd_values = q_des
dqd_values = qd_des
ddqd_values = qdd_des

# Plotar os resultados
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(t_span, qd_values[:, 0], label='qd[0]')
plt.plot(t_span, qd_values[:, 1], label='qd[1]')
plt.plot(t_span, qd_values[:, 2], label='qd[2]')
plt.ylabel('Posição Desejada (qd)')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(t_span, dqd_values[:, 0], label='dqd[0]')
plt.plot(t_span, dqd_values[:, 1], label='dqd[1]')
plt.plot(t_span, dqd_values[:, 2], label='dqd[2]')
plt.ylabel('Velocidade Desejada (dqd)')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(t_span, ddqd_values[:, 0], label='ddqd[0]')
plt.plot(t_span, ddqd_values[:, 1], label='ddqd[1]')
plt.plot(t_span, ddqd_values[:, 2], label='ddqd[2]')
plt.xlabel('Tempo (s)')
plt.ylabel('Aceleração Desejada (ddqd)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

def dynamics(t, y, t_des, q_des, qd_des, qdd_des, Kp, Kd, Ki):
    # Extrai os estados atuais
    q = y[0:3]
    dq = y[3:6]
    integral_e = y[6:9] # O erro integral é agora parte do estado

    # Encontra os valores desejados no tempo t usando interpolação
    # np.interp requer que t_des seja sorted e unique, o que mstraj fornece.
    qd_t = np.array([np.interp(t, t_des, q_des[:, i]) for i in range(q_des.shape[1])])
    dqd_t = np.array([np.interp(t, t_des, qd_des[:, i]) for i in range(qd_des.shape[1])])
    # ddqd_t = np.array([np.interp(t, t_des, qdd_des[:, i]) for i in range(qdd_des.shape[1])]) # Não usado no PID básico

    # Calcula os erros
    e = qd_t - q
    de = dqd_t - dq

    # Calcula o torque de controle PID
    tau = Kp * e + Kd * de + Ki * integral_e

    # Dinâmica do robô (simplificada)
    M = np.diag([0.05, 1.0, 0.02])  # Inércia
    C = np.diag([0.01, 0.1, 0.01])  # Atrito
    G = np.zeros(3)  # Gravidade (considerada zero neste caso)

    # Calcula a aceleração das juntas
    # M * ddq = tau - C * dq - G
    ddq = np.linalg.inv(M) @ (tau - C @ dq - G)

    # A derivada do erro integral é o próprio erro de posição
    de_integral = e

    # Derivadas do vetor de estado [q, dq, integral_e] são [dq, ddq, e]
    dy = np.concatenate([dq, ddq, de_integral])
    return dy

def run_pid_control():
    # Condições iniciais: [q_inicial, dq_inicial, integral_e_inicial]
    # q_inicial = posição inicial (primeiro ponto da trajetória desejada)
    # dq_inicial = velocidade inicial (geralmente zero)
    # integral_e_inicial = erro integral inicial (geralmente zero)
    # A trajetória desejada começa em q_des[0, :], então a posição inicial ideal é essa.
    # Para fins de demonstração, vamos começar em zeros e ver o controlador corrigir.
    y0 = np.zeros(9)  # Novo estado: [q, dq, integral_e]

    # Vetor de tempo para a simulação (usar os tempos da trajetória desejada ou um intervalo similar)
    # Usando t_des diretamente para que os pontos de tempo da simulação correspondam aos da trajetória desejada,
    # o que simplifica a comparação e o lookup (embora interpolação seja mais geral).
    # Se quisermos simular em um intervalo de tempo diferente ou mais denso, usaríamos np.linspace
    # dentro do span de t_des. Vamos usar t_des por simplicidade agora.
    t_span_sim = [t_des[0], t_des[-1]] # Intervalo de tempo para solve_ivp
    t_eval_sim = t_des # Pontos de tempo onde queremos a solução

    # Simulação usando solve_ivp
    # Passando os dados da trajetória e os ganhos PID como argumentos adicionais
    sol = solve_ivp(dynamics,
                    t_span_sim,
                    y0,
                    t_eval=t_eval_sim,
                    args=(t_des, q_des, qd_des, qdd_des, Kp, Kd, Ki))

    # Extrai os resultados da simulação
    q_sim = sol.y[0:3]  # Trajetória simulada (posição)
    dq_sim = sol.y[3:6] # Velocidade simulada
    integral_e_sim = sol.y[6:9] # Erro integral simulado
    t_sim = sol.t # Tempos da simulação

    # Comparando com a trajetória desejada
    # Como usamos t_des para t_eval_sim, podemos usar q_des diretamente
    # Se t_eval_sim fosse diferente, precisaríamos interpolar q_des em t_sim
    qd_traj = q_des.T # Transpor q_des para ter juntas nas linhas e tempo nas colunas

    # Calcular a velocidade desejada interpolada nos tempos de simulação
    # Corrigido para interpolar para todos os tempos t_sim para cada junta
    dqd_traj_sim = np.array([np.interp(t_sim, t_des, qd_des[:, i]) for i in range(qd_des.shape[1])])
    # dqd_traj_sim agora tem forma (num_juntas, num_tempos_simulacao)


    # Plotando os resultados
    plt.figure(figsize=(10, 12))

    # Plot da Posição das Juntas
    plt.subplot(3, 1, 1)
    for i in range(3):
        plt.plot(t_sim, q_sim[i, :], label=f'q{i+1} Simulado', linewidth=1.5)
        plt.plot(t_des, qd_traj[i, :], '--', label=f'q{i+1} Desejado', linewidth=1.2)
    plt.ylabel('Posição [rad ou m]')
    plt.title('Simulação do Controle PID')
    plt.legend()
    plt.grid(True)

    # Plot da Velocidade das Juntas
    plt.subplot(3, 1, 2)
    # Usar o dqd_traj_sim calculado corretamente
    for i in range(3):
        plt.plot(t_sim, dq_sim[i, :], label=f'dq{i+1} Simulado', linewidth=1.5)
        plt.plot(t_sim, dqd_traj_sim[i, :], '--', label=f'dq{i+1} Desejado', linewidth=1.2)
    plt.ylabel('Velocidade [rad/s ou m/s]')
    plt.xlabel('Tempo [s]')
    plt.legend()
    plt.grid(True)

     # Plot do Erro de Posição
    plt.subplot(3, 1, 3)
    # O erro é calculado como qd_traj - q_sim.
    # qd_traj tem forma (num_juntas, num_tempos_desejados)
    # q_sim tem forma (num_juntas, num_tempos_simulacao)
    # Como t_eval_sim = t_des, num_tempos_simulacao = num_tempos_desejados.
    # Então a subtração direta funciona.
    error = qd_traj - q_sim
    for i in range(3):
        plt.plot(t_sim, error[i, :], label=f'Erro q{i+1}', linewidth=1.5)
    plt.ylabel('Erro de Posição [rad ou m]')
    plt.xlabel('Tempo [s]')
    plt.legend()
    plt.grid(True)


    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    run_pid_control()
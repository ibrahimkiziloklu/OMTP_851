from dmp import dmp_cartesian, dmp_joint
import numpy as np
import pandas as pd
import quaternion
import matplotlib.pyplot as plt

def omega_to_quat(omega, dt):
    # integrate to get quaternion for orientation
    half_omega_dt = 0.5 * omega * dt
    omega_quat = np.exp(quaternion.quaternion(0, half_omega_dt[0], half_omega_dt[1], half_omega_dt[2]))
    return omega_quat


if __name__ == '__main__':
    demo_filename = "demonstration.csv"

    # Read Demonstration and generate time vector
    demo = pd.read_csv(demo_filename, delimiter=" ")

    q = demo[['actual_q_0', 'actual_q_1', 'actual_q_2', 'actual_q_3', 'actual_q_4', 'actual_q_5']].to_numpy()
    p = demo[['actual_TCP_pose_0', 'actual_TCP_pose_1', 'actual_TCP_pose_2']].to_numpy()
    aa = demo[['actual_TCP_pose_3',	'actual_TCP_pose_4', 'actual_TCP_pose_5']].to_numpy()

    dt = 0.0001
    tau = len(demo) * dt
    ts = np.arange(0, tau, dt)

    # Joint-Space DMP
    cs_alpha = -np.log(0.0001)

    ## setup the joint space DMP structure
    dmp_joint_space = dmp_joint.JointDMP(NDOF=6, n_bfs=90, alpha=100, beta=50, cs_alpha=cs_alpha)

    ## DMp train
    dmp_joint_space.train(q, ts, tau)

    ## DMP integrate
    q_out, dq_out, ddq_out = dmp_joint_space.rollout(ts, tau , FX = True)
 
    ## Plot the data
    fig1, axs = plt.subplots(3, 2)
    axs[0,0].plot(ts, q[:, 0], label='Demo q1')
    axs[0,0].plot(ts, q_out[:, 0], label='DMP q1')
    axs[0,0].legend()
    axs[1,0].plot(ts, q[:, 1], label='Demo q2')
    axs[1,0].plot(ts, q_out[:, 1], label='DMP q2')
    axs[1,0].legend()
    axs[2,0].plot(ts, q[:, 2], label='Demo q3')
    axs[2,0].plot(ts, q_out[:, 2], label='DMP q3')
    axs[2,0].legend()
    axs[0, 1].plot(ts, q[:, 3], label='Demo q4')
    axs[0, 1].plot(ts, q_out[:, 3], label='DMP q4')
    axs[0, 1].legend()
    axs[1, 1].plot(ts, q[:, 4], label='Demo q5')
    axs[1, 1].plot(ts, q_out[:, 4], label='DMP q5')
    axs[1, 1].legend()
    axs[2, 1].plot(ts, q[:, 5], label='Demo q6')
    axs[2, 1].plot(ts, q_out[:, 5], label='DMP q6')
    axs[2, 1].legend()
    plt.suptitle('Joint-space DMP')


    ## Cartesian DMP
    # Ensure that the orientations are formatted properly
    for i in range(1, len(aa)):
        if np.dot(aa[i], aa[i - 1]) < 0:
            aa[i] *= -1

    quats = quaternion.from_rotation_vector(aa)

    # Ensure that the quaternions do not flip sign
    for i in range(1, len(quats)):
        q0 = quats[i - 1]
        q1 = quats[i]

        if np.dot(q0.vec, q1.vec) < 0:
            quats[i] *= -1

    # Train DMP
    cs_alpha = -np.log(0.0001)
    dmp = dmp_cartesian.CartesianDMP(n_bfs=20, alpha=48, beta=48/4, cs_alpha=cs_alpha)
    dmp.train(p, quats, ts, tau)

    p_out, dp_out, ddp_out, q_out, omega_out, d_omega_out = dmp.rollout(ts, tau)
    aa_out = quaternion.as_rotation_vector(q_out)

    quats_array = quaternion.as_float_array(quats)
    quats_out_array = quaternion.as_float_array(q_out)

    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d')
    ax1.plot3D(p[:, 0], p[:, 1], p[:, 2], label=r'Demonstration', linewidth=2)
    ax1.plot3D(p_out[:, 0], p_out[:, 1], p_out[:, 2], label=r'DMP', linewidth=2)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    plt.title('Cartesian-space DMP (Position)')

    fig2, axs = plt.subplots(3, 1)
    axs[0].plot(ts, p[:, 0], label='Demo x')
    axs[0].plot(ts, p_out[:, 0], label='DMP x')
    axs[0].legend()
    axs[1].plot(ts, p[:, 1], label='Demo y')
    axs[1].plot(ts, p_out[:, 1], label='DMP y')
    axs[1].legend()
    axs[2].plot(ts, p[:, 2], label='Demo z')
    axs[2].plot(ts, p_out[:, 2], label='DMP z')
    axs[2].legend()
    plt.suptitle('Cartesian-space DMP (Position)')

    fig3, axs = plt.subplots(4, 1)
    axs[0].plot(ts, quats_array[:, 0], label='Demo qx')
    axs[0].plot(ts, quats_out_array[:, 0], label='DMP qx')
    axs[0].legend()
    axs[1].plot(ts, quats_array[:, 1], label='Demo qy')
    axs[1].plot(ts, quats_out_array[:, 1], label='DMP qy')
    axs[1].legend()
    axs[2].plot(ts, quats_array[:, 2], label='Demo qz')
    axs[2].plot(ts, quats_out_array[:, 2], label='DMP qz')
    axs[2].legend()
    axs[3].plot(ts, quats_array[:, 3], label='Demo qz')
    axs[3].plot(ts, quats_out_array[:, 3], label='DMP qz')
    axs[3].legend()
    plt.suptitle('Cartesian-space DMP (Quaternion)')

    plt.show()
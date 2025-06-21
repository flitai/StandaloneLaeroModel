import pandas as pd
import matplotlib.pyplot as plt
import sys

def analyze_flight_data(filename='maneuver_log.csv'):
    """
    读取并分析飞行仿真日志文件，生成性能图表。
    """
    # --- 1. 读取CSV数据 ---
    try:
        df = pd.read_csv(filename)
        print(f"成功读取日志文件: {filename}")
        print(f"数据包含 {len(df)} 行, {len(df.columns)} 列。")
    except FileNotFoundError:
        print(f"错误: 文件 '{filename}' 未找到。")
        print("请先运行C++仿真程序以生成日志文件。")
        sys.exit(1)

    # --- 2. 绘制飞行轨迹图 (X-Y平面) ---
    plt.figure(figsize=(10, 10))
    plt.plot(df['PosX'], df['PosY'], label='Actual Trajectory', color='blue', linewidth=2)
    plt.plot(df['TargetPosX'], df['TargetPosY'], label='Target Trajectory', color='red', linestyle='--', linewidth=1.5)
    plt.title('Flight Trajectory (Top-Down View)')
    plt.xlabel('East Position (m)')
    plt.ylabel('North Position (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal') # 确保X和Y轴比例相同，避免轨迹变形
    plt.show()

    # --- 3. 绘制高度剖面图 ---
    plt.figure(figsize=(14, 7))
    plt.plot(df['Time'], df['Alt'], label='Actual Altitude', color='blue', linewidth=2)
    plt.plot(df['Time'], df['TargetAlt'], label='Target Altitude', color='red', linestyle='--', linewidth=1.5)
    plt.title('Altitude Profile Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.legend()
    plt.grid(True)
    plt.show()

    # --- 4. 绘制误差变化图 (使用子图) ---
    fig, axs = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle('Tracking Errors Over Time', fontsize=16)

    # 位置误差
    axs[0].plot(df['Time'], df['ErrorDist'], color='purple')
    axs[0].set_ylabel('Position Error (m)')
    axs[0].grid(True)

    # 高度误差
    axs[1].plot(df['Time'], df['ErrorAlt'], color='green')
    axs[1].set_ylabel('Altitude Error (m)')
    axs[1].grid(True)

    # 航向误差
    axs[2].plot(df['Time'], df['ErrorHdg'], color='orange')
    axs[2].set_ylabel('Heading Error (deg)')
    axs[2].grid(True)

    # 速度误差
    axs[3].plot(df['Time'], df['ErrorVel'], color='brown')
    axs[3].set_ylabel('Velocity Error (kts)')
    axs[3].set_xlabel('Time (s)')
    axs[3].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96]) # 调整布局以适应主标题
    plt.show()

if __name__ == '__main__':
    # 如果您想分析其他文件，可以在命令行中传入文件名
    # 例如: python analyze_trajectory.py my_other_log.csv
    if len(sys.argv) > 1:
        analyze_flight_data(sys.argv[1])
    else:
        analyze_flight_data()
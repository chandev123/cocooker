import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

def draw_graphs(csv_file='pot_pick_data.csv'):
    try:
        # 1. 데이터 불러오기
        df = pd.read_csv(csv_file)
        print(f"'{csv_file}' 로드 완료. 총 {len(df)}건의 데이터가 있습니다.")
        
        # 2. 캔버스 설정 (2x2)
        sns.set_style("whitegrid")
        fig, axs = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'Pot Pick Test Analysis (Total: {len(df)})', fontsize=16)

        # [1] 성공률 (Bar Chart)
        success_counts = df['Success'].value_counts()
        success_val = success_counts.get(1, 0)
        fail_val = success_counts.get(0, 0)
        
        axs[0, 0].bar(['Success', 'Fail'], [success_val, fail_val], color=['green', 'red'])
        axs[0, 0].set_title(f'Success Rate: {success_val}/{len(df)} ({success_val/len(df)*100:.1f}%)')
        axs[0, 0].set_ylabel('Count')
        for i, v in enumerate([success_val, fail_val]):
            axs[0, 0].text(i, v, str(v), ha='center', va='bottom', fontsize=12)

        # [2] 시간 분석 (Line Chart)
        axs[0, 1].plot(df['Trial'], df['Detect_Time'], 'o-', label='Detect Time')
        axs[0, 1].plot(df['Trial'], df['Motion_Time'], 'x-', label='Motion Time')
        axs[0, 1].set_title('Time Analysis (Seconds)')
        axs[0, 1].set_xlabel('Trial')
        axs[0, 1].set_ylabel('Time (s)')
        axs[0, 1].legend()

        # [3] 신뢰도 추이 (Line Chart)
        axs[1, 0].plot(df['Trial'], df['Confidence'], 's--', color='purple')
        axs[1, 0].set_title('YOLO Confidence Score')
        axs[1, 0].set_xlabel('Trial')
        axs[1, 0].set_ylabel('Confidence')
        axs[1, 0].set_ylim(0, 1.1)

        # [4] 혼동 행렬 (Heatmap)
        # 실제로는 "집기 시도(Positive)"만 있으므로, 
        # TP(성공)와 FP(실패)만 존재함.
        conf_matrix = [[success_val, fail_val]]
        sns.heatmap(conf_matrix, annot=True, fmt='d', cmap='Blues', ax=axs[1, 1],
                    cbar=False, annot_kws={"size": 20})
        axs[1, 1].set_title('Result Matrix (Detected -> Picked)')
        axs[1, 1].set_xticks([0.5, 1.5])
        axs[1, 1].set_xticklabels(['Success', 'Fail'])
        axs[1, 1].set_yticks([])

        plt.tight_layout()
        save_name = 'final_analysis_result.png'
        plt.savefig(save_name)
        print(f"분석 그래프가 '{save_name}'으로 저장되었습니다!")
        plt.show()

    except FileNotFoundError:
        print(f"오류: '{csv_file}' 파일을 찾을 수 없습니다. 같은 폴더에 있는지 확인해주세요.")
    except Exception as e:
        print(f"오류 발생: {e}")

if __name__ == "__main__":
    draw_graphs()
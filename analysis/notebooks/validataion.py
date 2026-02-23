import numpy as np
import pandas as pd  # 누락된 pandas 추가
import matplotlib.pyplot as plt
from numpy.linalg import norm
import glob

# ==========================================
# 1. 데이터 로드 및 전처리
# ==========================================
file_pattern = '*.csv'
all_files = glob.glob(file_pattern)

if len(all_files) > 0:
    df_list = []
    for filename in all_files:
        print(f"📄 Reading {filename}...")
        df_list.append(pd.read_csv(filename))
    df = pd.concat(df_list, ignore_index=True)
    print(f"✅ 총 {len(all_files)}개 파일 로드 성공!")
else:
    print("❌ CSV 파일 없음. 더미 데이터를 생성합니다.")
    # 테스트용 더미 데이터
    np.random.seed(42) # 결과 재현을 위해 시드 고정
    data = {
        'Trial': range(1, 101),
        'Detect_Time(ms)': np.random.normal(120, 15, 100),
        'Motion_Time(ms)': np.random.normal(150, 20, 100) # 비교를 위해 값 범위를 조정함
    }
    df = pd.DataFrame(data)

# 중요: NaN(빈 값)이 있으면 계산이 안 되므로 0으로 채우거나 해당 행을 삭제해야 합니다.
df = df.fillna(0)

# ==========================================
# 2. 분석할 두 벡터 선택 (여기를 수정해서 사용하세요)
# ==========================================
# 예시: 'Detect_Time' 패턴과 'Motion_Time' 패턴이 얼마나 유사한가?
# 데이터가 100개 행이라면 100차원 벡터가 됩니다.
vec_a = df['Detect_Time(ms)'].values
vec_b = df['Motion_Time(ms)'].values

# ==========================================
# 3. 코사인 유사도 계산
# ==========================================
cosine_sim = np.dot(vec_a, vec_b) / (norm(vec_a) * norm(vec_b))
print(f"📊 두 데이터 컬럼 간의 코사인 유사도: {cosine_sim:.4f}")

# ==========================================
# 4. 시각화 (차원이 높으므로 산점도로 표현)
# ==========================================
# 데이터가 많을 때는 화살표(Quiver)보다 산점도(Scatter)가 관계를 보기 좋습니다.
# 원점에서 뻗어나가는 경향성을 봅니다.

plt.figure(figsize=(6, 6), dpi=100) # 해상도/크기 설정 유지

plt.scatter(vec_a, vec_b, alpha=0.6, color='purple', label='Data Points')

# 기준선 (y=x): 데이터가 이 선 위에 모일수록 유사도가 1에 가까움
max_val = max(vec_a.max(), vec_b.max())
plt.plot([0, max_val], [0, max_val], 'r--', label='Perfect Similarity (y=x)')

plt.title(f"Correlation / Similarity Check\nCosine Sim: {cosine_sim:.4f}")
plt.xlabel("Detect Time (ms)")
plt.ylabel("Motion Time (ms)")
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend()

plt.show()
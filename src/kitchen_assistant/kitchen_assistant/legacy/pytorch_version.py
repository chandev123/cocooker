import torch

# 1. 내 GPU (RTX 5080)의 실제 능력치 (Compute Capability) 확인
print(f"내 GPU 아키텍처: {torch.cuda.get_device_capability()}")

# 2. 지금 깔린 PyTorch가 지원 가능한 아키텍처 리스트 확인
print(f"라이브러리 지원 목록: {torch.cuda.get_arch_list()}")
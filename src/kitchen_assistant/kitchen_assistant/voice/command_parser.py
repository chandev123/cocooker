from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple

OBJ_SYNONYMS = {
    "cup": ["cup", "컵"],
    "ramen": ["ramen", "라면"],
    "scissors": ["scissors", "가위", "주방가위"],
    "spatula": ["spatula", "뒤집개"],
    "spoon": ["spoon", "국자", "숟가락"],
    "knife": ["knife", "칼"],
    "pan": ["pan", "프라이팬", "팬"],
    "pot": ["pot", "냄비"],
    "apple": ["apple", "사과"],
    "orange": ["orange", "오렌지"],
    "bottle": ["bottle", "콜라", "페트병", "병"],
}

RELEASE_WORDS = [
    "이제 줘", "줘", "놔", "놓아도 돼", "놓아", "주세요", "그리퍼 열어", "열어", "열어줘", "그립퍼 열어"
]

REST_WORDS = [
    "이제 쉬어", "쉬어도 돼", "쉬어", "대기", "쉬자"
]

CONJ_WORDS = ["와", "과", "랑", "하고", "및", "그리고", "그리구"]

@dataclass
class ParseResult:
    kind: str  # "plan" | "release" | "rest" | "unknown"
    plan: List[str] = None

def _find_objects_in_text(text: str) -> List[str]:
    hits: List[Tuple[int,str]] = []
    for obj, syns in OBJ_SYNONYMS.items():
        for s in syns:
            idx = text.find(s)
            if idx >= 0:
                hits.append((idx, obj))
                break
    hits.sort(key=lambda x: x[0])
    plan: List[str] = []
    for _, obj in hits:
        if obj not in plan:
            plan.append(obj)
    return plan

def is_release_cmd(text: str) -> bool:
    return any(w in text for w in RELEASE_WORDS)

def is_rest_cmd(text: str) -> bool:
    return any(w in text for w in REST_WORDS)

def parse_command(text: str) -> ParseResult:
    t = (text or "").strip()
    if not t:
        return ParseResult(kind="unknown", plan=[])

    if is_rest_cmd(t):
        return ParseResult(kind="rest", plan=[])

    if is_release_cmd(t):
        return ParseResult(kind="release", plan=[])

    base_objs = _find_objects_in_text(t)
    plan = list(base_objs)

    # recipe rules
    if ("라면" in t or "ramen" in t) and ("요리" in t or "끓" in t or "조리" in t):
        if "ramen" not in plan:
            plan.insert(0, "ramen")
        if "pot" not in plan:
            plan.append("pot")

    if (("계란" in t) or ("후라이" in t) or ("고기" in t)) and ("요리" in t or "굽" in t or "부치" in t):
        if "pan" not in plan:
            plan.append("pan")
        if "spatula" not in plan:
            plan.append("spatula")

    if ("콜라" in t or "bottle" in t) and (("컵" in t) or ("따라" in t) or ("마시" in t) or ("조금만" in t)):
        if "bottle" not in plan:
            plan.append("bottle")
        if "cup" not in plan:
            plan.append("cup")

    if ("사과" in t or "apple" in t) and ("깎" in t or "자르" in t):
        if "apple" not in plan:
            plan.append("apple")
        if "knife" not in plan:
            plan.append("knife")

    if any(w in t for w in CONJ_WORDS) and len(plan) >= 2:
        return ParseResult(kind="plan", plan=plan)

    if len(plan) >= 1:
        return ParseResult(kind="plan", plan=plan)

    return ParseResult(kind="unknown", plan=[])

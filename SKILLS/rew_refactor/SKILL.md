---
name: rew_refactor
description: "Use when the user asks for refactor, rewrite, replacement, simplification, legacy removal, deleting fallback paths, or aggressively reducing complexity in a coding task. Applies to code generation tasks where old implementations should be removed instead of preserved."
user_invocable: true
version: 0.1
---

# /rew_refactor

## 用途

用于约束代码生成风格：当任务本质上是重构、改写、替换、清理旧实现时，优先用单一路径完成目标，不为旧逻辑保留 fallback。

## 核心规则

- 按需求大刀阔斧修改，不保留含糊的过渡层。
- 旧函数、旧分支、旧适配层、旧配置如果已经被新方案取代，直接删除。
- 不默认保留 fallback、兼容分支、双轨实现、影子逻辑、迁移垫片。
- 以降低系统复杂性为优先目标，宁可做清晰的替换，也不要做模糊的兼容。
- 如果旧 API 只是为了照顾仓库内部旧代码，优先同步修改调用方，而不是保留旧 API。
- 不保留“以后可能有用”的死代码、注释掉的旧实现、无主开关。

## 决策准则

- 用户明确要重构或替换时，默认新实现是唯一主路径。
- 如果一个兼容层只会让系统更复杂，就删除它。
- 如果一个辅助函数只服务于即将删除的旧逻辑，就一并删除或内联。
- 如果是否还能删除某段旧代码存在疑问，先做最便宜的局部验证，再删掉被新路径替代的部分。

## 输出要求

- 直接输出代码文件或 patch。
- 在同一轮改动中尽量完成“新增替代实现 + 删除旧实现 + 更新调用方”。
- 首次实质性修改后，优先做一次与改动范围直接相关的验证。

## 禁止行为

- 不要默认加 fallback 兜底。
- 不要为了“安全感”保留废弃 wrapper。
- 不要把旧代码注释留在文件里。
- 不要为了兼容仓库内部旧写法而维持双轨路径。

## 适用范围

所有 agent 编码任务中，只要目标更接近“重构/替换/删旧逻辑”，就应优先采用本 skill 的规则。

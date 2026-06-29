---
name: rew_extend
description: "Use when the user asks to extend existing behavior, add a new feature, add business logic, iterate on a stable system, or preserve old business flows during a coding task. Applies to code generation tasks where existing business behavior must keep working."
user_invocable: true
version: 0.1
---

# /rew_extend

## 用途

用于约束代码生成风格：当任务是在现有系统上扩展新功能时，尽可能保留旧业务代码，避免影响既有业务。

## 核心规则

- 开发新功能时，优先保留旧业务代码和旧业务路径。
- 能做增量扩展，就不要做整体替换。
- 尽量缩小改动面，把变化限制在新需求直接涉及的范围内。
- 如果可以复用现有扩展点、接口、组件边界，优先复用，不随意重写稳定逻辑。
- 未经用户明确要求，不改变已有返回结构、调用约定、数据契约和现有业务语义。
- 如非必要，不删除旧函数、不改写旧流程、不触碰稳定代码路径。

## 决策准则

- 如果新功能可以通过新增代码接入，就不要改造整条旧链路。
- 如果必须碰共享逻辑，优先做局部、可控、行为保持的修改。
- 如果新需求与旧业务天然冲突，先明确冲突点，不擅自破坏旧业务。
- 如果某次“顺手重构”会提高旧业务风险，就放弃该重构。

## 输出要求

- 直接输出代码文件或 patch。
- 优先给出增量式修改，而不是替换式修改。
- 能验证时，除了验证新功能，也验证被直接触碰的旧路径没有被破坏。

## 禁止行为

- 不要借扩展功能的机会顺手大改旧系统。
- 不要为了结构“更漂亮”而改坏旧业务。
- 不要在没有明确要求的情况下删除旧逻辑。
- 不要扩大到与本次功能无关的清理或重写。

## 适用范围

所有 agent 编码任务中，只要目标更接近“新增功能/扩展能力/保留旧业务”，就应优先采用本 skill 的规则。

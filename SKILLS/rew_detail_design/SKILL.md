---
name: rew_detail_design
description: "Use when the user asks for a detailed design document, an implementation plan, a technical spec, interface/type design, or a hand-off plan instead of code. Produces an exhaustive, decision-free design in which every type, identifier and per-function implementation step is fully determined, and changes no code."
user_invocable: true
version: 0.1
---

# /rew_detail_design

## 核心要求

使用本Skill时**不修改、不落地任何代码**，只产出一份可直接开工的详细设计计划书：

- 新定义的类型、函数、对象的命名需要确定好，函数的实现也用文字描述出来。
- 足够详细，不含模糊或模棱两可的描述：任何水平的程序员或 AI Agent 拿到它，都不需要再怎么思考就能开始编码、落地，且实现出来的效果基本一致。
- 必要时可以包含少量伪代码。
- 使用者同时开着计划模式时，同样要产出这份详细设计，见「计划模式补充」。

## 用途

把「需求 → 方案 → 命名 → 类型 → 函数实现步骤 → 验证方式」一次性全部确定下来，产出一份详细设计计划书，交给后续实现者照着写代码。
详细计划书的读者是**执行者，不是设计者**：他不掌握本次对话的上下文，只会照文档写代码，不会也不应补做任何设计判断。


## 硬约束

- 禁止 Edit/Write 源码、工程文件、生成物，禁止构建、运行测试、执行 git 写操作。
- 详细计划书中每条「现状」都必须先核实（读过对应文件），引用既有符号时给出 `文件路径:行号`。无法核实的，写成疑问向用户求证，不得凭印象编写。
- 需求里没有、又无法由现状推导的决策属于用户所有：先问，不要替用户拍板

## 模糊写法黑名单

计划书中不能出现下列写法，必须改成确定表述：

- `等`、`等等`、`诸如……之类`
- 用于替代明确说明的 `类似`、`参考现有实现`（未指明文件与函数）
- `视情况`、`酌情`、`按需`、`适当`、`合理的`、`必要时`
- `可以考虑`、`建议`、`最好`、`尽量`、`原则上`
- `大概`、`若干`、`一些`、`多个`（未给出数量或判定条件）
- `TODO`、`TBD`、`待定`、`后续补充`、`具体实现略`
- `自行决定`、`自由发挥`、`按惯例`


## 计划模式补充

- 计划模式和本Skill不冲突，应该先生成Harness要求的普通计划，然后再生成本Skill要求的详细设计计划。  


## 与其他Skill的关系

本Skill只出设计，不落地。进入实现阶段后，按任务性质使用 rew_extend 或 rew_refactor，并遵守 rew_style 的代码风格；这些风格要点可提前写进计划书的禁止改动清单，减少实现阶段的偏差。

## 适用范围

用户要求「先出详细设计 / 先给实施计划」时使用。用户要求直接改代码时，不使用本Skill。

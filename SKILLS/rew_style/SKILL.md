---
name: rew_style
description: "Use when the user wants code generation to follow a strict house style: use precise action-based identifier names, add brief Chinese XML doc comments to newly added functions, avoid extracting one-off short functions, keep logic simple and concise, and split larger blocks with single-line comments."
user_invocable: true
version: 0.1
---

# /rew_style

## 用途

用于约束代码生成风格：让生成的代码遵守固定的注释习惯、函数拆分习惯和简洁实现偏好。

## 核心规则

- 减少明显无意义的抽象，保持代码逻辑简单，避免不必要的复杂度。（如果某段实现可以用更直接的写法完成，就不要引入额外抽象。） 
- 如果提取辅助函数只会让代码更碎、且没有复用价值，就保留在当前上下文中。只有很长的且明确需要复用的逻辑，才拆出来作为单独的函数。  
- 实现应尽量简洁，不写冗长、绕弯或过度铺陈的代码。

## 禁止行为

- 不要为了形式统一而把短小的一次性逻辑拆成很多函数。
- 不要引入与需求无关的复杂设计、抽象层或模板化结构。
- 不要输出大段连续且没有任何语义注释的新增代码。
- 不要写过长的说明性代码或重复表达同一层逻辑。
- 在计划模式下，不要跳过关键澄清，也不要在类名、结构体名等核心命名尚未确认时直接开工。

## 标识符命名

- 命名时尽量少用 `Ensure`、`Normalize` 这类泛词：`Ensure` 优先改成 `Check`、`Validate` 或更具体动作；`Normalize` 仅用于几何/图形学含义的归一化，其他路径、文本、资源场景用更具体的命名。
- 项目自定义标识符禁止使用宽泛的 `Resolve` 作为兜底动词。命名前必须确认真实动作：加载资源使用 `Load`，获取已有对象使用 `Get`，解析文本或序列化数据使用 `Parse`，读取外部数据使用 `Read`，类型或表示转换使用 `Convert` 或 `To`，查找目标使用 `Find`；同时包含获取和加载时使用 `GetOrLoad` 等能完整表达行为的组合名称。
- 函数包含多步操作时，名称必须体现主要结果或可观察副作用，不能用 `Resolve` 隐藏其实际行为。例如使用 `LoadBuiltinShaders`，不要使用 `ResolveBuiltinResources`。
- 不要把 `Make` 当默认兜底动词；除非确实是在构造新值，否则优先用 `Get`、`To`、`Build`、`Create`、`Write`、`Find`、`Check`、`Validate` 等更具体动作。  


## 注释  

- 新增函数应附带对应注释，长逻辑段应做分块注释。
- 避免出现大段没有注释分隔的代码块。  

代码注释示例：
```
//public 函数使用单行注释  
public void Foo(){}
//private 函数也使用单行注释  
private void Bar(){}
```  

```csharp
/// <summary> C# public 函数使用单行 summary 注释 </summary>
public void Foo(){}
```


## 计划模式补充规则

- 在计划模式下，不要急于开始实现；优先通过提问把需求和技术方案讨论清楚。
- 在计划模式下，应尽可能多地提出有价值的问题，避免过早基于模糊前提做决定。
- 对实现相关的关键技术细节，必须逐项与用户确认，例如接口形态、数据流、生命周期、错误处理、性能取舍、兼容性边界和测试预期。
- 对命名相关细节，要细致确认到类名、结构体名、函数名、字段名和文件名；不要自行拍板看起来“差不多”的命名。
- 一个新增的较大模块/功能，新的类定义/结构体定义/函数定义放在哪个代码文件，也需要用户确认。
- 如果存在多个合理方案，不要直接替用户决定；应先明确列出差异、影响和推荐项，再请用户确认。
- 如果用户给出的信息还不足以支撑稳定实现，应继续追问，直到关键歧义被消除。
- 在计划模式下，输出的计划应建立在已确认的信息之上；未确认的部分要明确标记，并继续向用户求证。
- 在计划模式下，先完成充分的问题澄清与命名确认，再进入实现或产出最终计划。


## 适用范围

所有 agent 编码任务中，只要用户希望生成结果遵守该编码风格，就应优先采用本 skill 的规则。

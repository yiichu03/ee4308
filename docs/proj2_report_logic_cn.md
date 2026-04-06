# `proj2_report_draft.md` 行文逻辑说明

本文不是报告正文，而是对当前报告草稿 [`proj2_report_draft.md`](./proj2_report_draft.md) 的“写作逻辑地图”。  
目的有两个：

- 让你快速看懂当前 draft 的叙事主线到底是什么。
- 让你在想修改时，能够直接指出“我想改哪一层逻辑”，而不是只能笼统地说“这里不太对”。

说明：

- 下面提到的行号，基于当前版本的 [`proj2_report_draft.md`](./proj2_report_draft.md)。
- 后续如果正文继续改动，行号可能会变化，但“段落功能”基本不变。

---

## 1. 这份 draft 的总主线是什么

当前 draft 的核心论点，可以压缩成一句话：

> 我们没有把 estimator 写成“做了很多小改动”的流水账，而是把它写成“针对几个明确失败模式，做了少量但有针对性的建模改进，并通过分阶段实验把最终方案筛出来”的故事。

再展开一点，就是：

1. 先界定范围：这份报告只讲 estimator，不讲 behavior/controller 的贡献归属。
2. 再定义问题：baseline 不是“整体都不行”，而是存在几个很具体的失败模式。
3. 再提出设计：每个保留下来的 estimator 改进，都要能对应回某个明确问题。
4. 再给实验：不是“参数调到最好就算”，而是要说明为什么保留、为什么删除、为什么最终参数这样选。
5. 最后用完整 run 收口：证明 estimator 不只是短窗口里看起来不错，而是能支撑完整任务直到落地后仍然稳定。

这个主线本质上是在迎合老师在 [`email.md`](./email.md) 里强调的偏好：

- 要讲 `why`
- 要讲 `how`
- 要有实验设计逻辑
- 失败尝试如果有分析价值，应该写
- 不要只是堆结果

---

## 2. 当前 draft 想“证明”的不是哪些事

这份 draft 有意避免去证明下面这些事情：

### 2.1 它不想证明“我们整个 proj2 都是我们写的”

因为你已经明确说了 `behavior` 和 `controller` 是队友写的。  
所以当前 draft 一开始就把范围切窄到 estimator，避免报告叙事和组内真实分工冲突。

### 2.2 它不想证明“我们每一个参数都找到了数学最优值”

因为现有实验链本身也不支持这种说法。  
当前 draft 采取的是更稳妥、更符合老师口味的逻辑：

> 我们不是声称“找到了全局最优参数”，而是声称“经过针对性实验，选择了更合理、更稳定、更能解释的参数组合”。

### 2.3 它不想证明“`x/y` 已经完美”

当前 draft 是故意留着这个“残余问题”的。  
因为老师更喜欢看到：

- 你知道系统哪里已经解决
- 也知道哪里还没完全解决
- 并且能解释为什么没完全解决

所以正文不是在“吹满分表现”，而是在强调：

- `z` 和 yaw 已经不是主要瓶颈
- 主要剩余问题是 planar lag / planar bias

这个逻辑是有意保留的，不是写漏了。

---

## 3. 整体结构为什么这样排

当前正文的章节顺序是：

1. `Abstract`
2. `Scope and Focus`
3. `Baseline Estimator and Failure Analysis`
4. `Final Estimator Design`
5. `Experimental Methodology`
6. `Results and Decisions`
7. `Final Full-Mission Validation`
8. `Discussion`
9. `Conclusion`
10. `Contribution Page Placeholder`

这个顺序不是随便排的，而是刻意避免两种常见坏写法：

### 3.1 避免“函数说明书式”写法

所谓函数说明书式写法，就是：

- `callbackSubIMU_()` 做了什么
- `callbackSubGPS_()` 做了什么
- `callbackSubBaro_()` 做了什么

这种写法的问题是：老师读完之后，只知道你“实现了功能”，但不知道你“为什么要这样改”。

当前 draft 是故意先讲问题，再讲设计。  
也就是说，技术细节不是凭空出现，而是作为问题的解法出现。

### 3.2 避免“结果堆砌式”写法

所谓结果堆砌式写法，就是：

- 贴很多表
- 贴很多 MAE
- 贴很多图

但没有说明这些实验为什么做、实验之间是什么关系、最后怎么形成定稿决策。

当前 draft 把结果按“决策链”排：

- 先证明某个 feature 值得保留
- 再证明某个 feature 值得删除
- 再证明参数怎么定
- 最后再用完整 run 收尾

也就是说，结果不是并列材料，而是有先后因果关系的。

---

## 4. 各章节在逻辑上分别承担什么功能

下面是最关键的部分：逐节解释当前 report 的“写作任务”。

## 4.1 `Abstract` 在做什么

对应正文大致位置：

- [`proj2_report_draft.md:13`](./proj2_report_draft.md#L13)
- [`proj2_report_draft.md:15`](./proj2_report_draft.md#L15)

这一段的功能不是“正式展开内容”，而是一次性给出整篇报告的四个锚点：

1. 只写 estimator，不写 behavior/controller
2. estimator 的核心问题是什么
3. 最终保留了哪些改进
4. 最终完整 run 的结果是什么

也就是说，`Abstract` 已经把全文缩成一句话摘要了。

如果你觉得这段要改，通常是改以下几种方向：

- 如果你想更“学术”，就把口语化的句子压缩得更短、更像论文摘要。
- 如果你想更“工程叙事”，就保留现在这种“问题 + 解决 + 结果”的结构。
- 如果你不想在摘要里提前放太多数字，可以删掉部分 MAE，只保留最关键一句结论。

这段不建议大改的点：

- “只写 estimator”的范围切割最好保留。
- “不是单次最好成绩，而是 ablation + repeated runs 定稿”最好保留。

---

## 4.2 `Scope and Focus` 为什么一定要放在前面

对应正文：

- [`proj2_report_draft.md:17`](./proj2_report_draft.md#L17)
- [`proj2_report_draft.md:37`](./proj2_report_draft.md#L37)

这节的作用不是技术，而是“给老师一个阅读框架”。

它完成三件事：

1. 对照 handout，说明 estimator 该做的东西都覆盖到了。
2. 明确说这份稿子只聚焦 estimator。
3. 明确说整篇将按“baseline -> failure -> retained changes -> rejected changes -> tuning -> final validation”来展开。

这节的重要性在于：

- 它帮你抢先定义“老师应该怎么读这篇报告”。
- 它也在替你解释为什么后面不写 behavior/controller。

如果你之后想调整全篇逻辑，这节通常最先动，因为它决定后面整篇的“阅读合同”。

比如你可以把它改成：

- 更偏 assignment checklist
- 更偏 estimator research note
- 更偏 engineering tuning report

但无论怎么改，这节都建议保留，因为它能大幅降低老师对“为什么只写 estimator”的困惑。

---

## 4.3 `Baseline Estimator and Failure Analysis` 是全文的“立论根基”

对应正文：

- [`proj2_report_draft.md:39`](./proj2_report_draft.md#L39)
- [`proj2_report_draft.md:73`](./proj2_report_draft.md#L73)

这一节是整篇最不能缺的一节之一。

它的写作任务是：

> 证明你的改动不是“为了加东西而加东西”，而是 baseline 的确存在特定问题，所以这些改动是必要的。

这里分成两个层次：

### 4.3.1 `2.1 Baseline structure`

对应：

- [`proj2_report_draft.md:41`](./proj2_report_draft.md#L41)
- [`proj2_report_draft.md:50`](./proj2_report_draft.md#L50)

它不是在炫耀设计，而是在先把 baseline 的“框架边界”钉住：

- 这是分轴滤波
- 不是 full 3D EKF
- 后续所有改动都发生在这个框架内

这很重要，因为它告诉老师：

- 你没有另起炉灶
- 你是在 handout 的简化模型上做针对性增强

### 4.3.2 `2.2 What went wrong in the early runs`

对应：

- [`proj2_report_draft.md:52`](./proj2_report_draft.md#L52)
- [`proj2_report_draft.md:73`](./proj2_report_draft.md#L73)

这段的逻辑是：

1. 先点出 `z` 轴的结构性问题
2. 用 `run1` 的早期 sweep 表说明“单纯调 variance 不够”
3. 再点出 `x/y` lag
4. 再点出 GPS timing mismatch

这相当于把全文后面要解决的问题先排出来了。

后面你看到的每一个 retained feature，都是在回应这里的某一条：

- sonar gating 对应 sonar 不可靠
- baro bias augmentation 对应 barometer 有 bias
- pseudo-velocity 对应 planar lag
- forward compensation 对应 timing mismatch

所以这一节本质上是“问题目录”。

如果你想改整篇报告的逻辑，最值得先改的就是这一节，因为它决定后面所有改动是不是“有来处”。

---

## 4.4 `Final Estimator Design` 为什么放在 failure analysis 后面

对应正文：

- [`proj2_report_draft.md:75`](./proj2_report_draft.md#L75)
- [`proj2_report_draft.md:194`](./proj2_report_draft.md#L194)

这个顺序是故意的：

- 不是先说“我做了什么”
- 而是先说“为什么必须做”

这样设计部分读起来才像“解决方案”，而不是“实现说明”。

### 4.4.1 这节的内部逻辑

这节的排列顺序也是有意设计的：

1. 先讲 prediction model  
   让老师知道 filter 基础骨架是什么。
2. 再讲 Joseph form  
   这是数值层的稳健性改进。
3. 再讲 retained correction models  
   说明观测模型怎么定义。
4. 再分别讲 sonar gating / baro bias / pseudo-velocity / forward compensation  
   让每个 feature 都对应一个独立设计动机。

换句话说，这一节是从“基础骨架”逐步走到“针对问题的增强逻辑”。

### 4.4.2 为什么这里要放公式

老师在邮件里明确提到，好的报告会“尽可能解释理论和方程”。

所以这里放公式不是为了论文味，而是为了说明：

- 你知道自己改的不是黑盒 heuristic
- 你知道每个量测模型和状态模型到底在数学上是什么意思

如果你觉得公式太多，可以减；  
但如果你完全删掉公式，这一节就会退化成“文字描述”，说服力会下降。

---

## 4.5 `Experimental Methodology` 不是配角，而是“防止老师不信”的一节

对应正文：

- [`proj2_report_draft.md:196`](./proj2_report_draft.md#L196)
- [`proj2_report_draft.md:240`](./proj2_report_draft.md#L240)

很多报告会把 methodology 写得很短，像附录。

当前 draft 没这么做，因为这节承担的是“实验可信度说明”。

它主要完成三件事：

1. 解释实验不是乱扫参数，而是问题导向。
2. 说明你用了哪些工具，所以实验是可复现的。
3. 给出最终 runtime 参数表，方便老师把后续实验结论和最终代码对上。

这节的隐藏逻辑是：

> 我们的参数不是拍脑袋来的，也不是挑最好的一次，而是按问题分层实验后定下来的。

如果你想让报告更“像工程报告”，这一节应该保留甚至加强。  
如果你想让报告更“像论文”，可以把工具列表压缩一些，但“实验设计哲学”那几句最好保留。

---

## 4.6 `Results and Decisions` 是整篇最强的“证据链”部分

对应正文：

- [`proj2_report_draft.md:242`](./proj2_report_draft.md#L242)
- [`proj2_report_draft.md:331`](./proj2_report_draft.md#L331)

这一节不是简单地“列结果”，而是按“决策顺序”组织的。

当前顺序是：

1. pseudo-velocity ablation
2. soft-gating rejection
3. forward-compensation ablation
4. planar parameter sweep
5. repeatability matters

这个顺序背后的逻辑是：

### 4.6.1 先证明 feature 值不值得保留

`5.1` 和 `5.3` 的作用是证明：

- pseudo-velocity 要留
- forward compensation 要留

也就是说，先确定“结构件”。

### 4.6.2 再证明什么 feature 要删掉

`5.2` 的作用不是补充，而是建立你的可信度：

> 我们不是把所有改动都往最终版本里塞，而是做过淘汰。

老师很吃这一套，因为这说明你不是“堆 feature”，而是在做 engineering tradeoff。

### 4.6.3 再做参数定稿

`5.4` 和 `5.5` 才开始处理“最终参数为什么这样选”。

这个顺序很关键，因为它在逻辑上表示：

- 先决定结构
- 再决定参数

这比“feature 和 parameter 混着讲”要清楚得多。

### 4.6.4 为什么最后一定要讲 repeatability

`5.5` 是我刻意保留的一个“老师会喜欢”的点。

因为很多学生会说：

- 这组参数最好

但没有说明：

- 是不是只是运气好的一次

这一节的作用就是把这个漏洞补上：

> 我们知道 run-to-run variance 存在，所以最终参数不是按单次最好，而是按更稳、更可解释的统计表现来选。

如果你想让报告更成熟，这节一定不能删。  
如果要压缩篇幅，也最多把表缩短，但不要删掉“repeatability matters”这个逻辑。

---

## 4.7 `Final Full-Mission Validation` 是“闭环收口”

对应正文：

- [`proj2_report_draft.md:333`](./proj2_report_draft.md#L333)
- [`proj2_report_draft.md:383`](./proj2_report_draft.md#L383)

这一节解决的是另一个层次的问题：

> 前面的实验大多是局部窗口和 ablation。那最终 estimator 放回完整 mission 里，能不能真的工作？

也就是说，这一节不是为了替代前面的 ablation，而是为了收尾。

### 4.7.1 为什么先解释 `gui_04` 的角色

`6.1` 先讲：

- 为什么 `gui_04` 比 `gui_03` 更适合作为最终 run
- 但为什么二者不能简单用一个 MAE 直接比较

这段是刻意防守式写法，作用是提前堵住两个可能的问题：

1. 你为什么换最终 baseline run？
2. 你是不是在拿不公平的统计窗口硬吹成绩？

也就是说，这一段是在替你建立“论证诚实性”。

### 4.7.2 `6.2` 为什么只给整体指标，不做复杂分解

这段的作用只是给出一个“最终结果快照”。

这里不做太复杂的分析，是因为更细的论证其实已经在前面的 ablation 和 repeatability 里完成了。  
这里的任务不是重新论证 feature，而是告诉老师：

- 最终整体效果是什么量级
- 哪些轴已经好
- 哪些轴仍然是残余问题

### 4.7.3 `6.3` 为什么强调 touchdown 之后

这是 `gui_04` 相比 `gui_03` 最有价值的新增逻辑：

> 不只是 landing 开始了，而是真的落地了，并且落地后 estimator 仍然稳定。

这段存在的意义有两个：

1. 证明完整 mission 真的跑完了。
2. 证明 estimator 在任务末端没有塌掉。

这个点其实很适合老师的口味，因为它不是“更好看的数字”，而是“更完整的工程证据”。

### 4.7.4 为什么这里一定要放图

这一节是最适合放图的，因为图在这里承担的是“全局证据”功能，而不是局部调参说明。

三张图分工很清楚：

- `trajectory_3d.png`：看完整任务几何形状
- `position_vs_time.png`：看轨迹随时间的整体一致性
- `error_vs_time.png`：看误差主要集中在哪些轴、哪个阶段

---

## 4.8 `Discussion` 是把“结果”翻译成“理解”

对应正文：

- [`proj2_report_draft.md:385`](./proj2_report_draft.md#L385)
- [`proj2_report_draft.md:403`](./proj2_report_draft.md#L403)

这节不是总结前文，而是在回答：

- 这些结果说明了什么？
- 我们真正学到了什么？

这里分成三块：

### 4.8.1 `What actually solved the problems`

这段的作用是防止报告看起来像“参数驱动的玄学调参”。  
它明确说：

- `z` 轴主要是 sensor modeling 问题
- `x/y` 主要是 lag 和 timing 问题
- Joseph form 是数值稳健性问题

这等于把全文重新抽象成三条“方法论结论”。

### 4.8.2 `What did not work`

这段让报告更像真实工程过程，而不是只保留“成功的故事”。

保留这段的意义是：

- 证明你确实做过取舍
- 也证明你知道为什么被删掉

### 4.8.3 `Remaining limitation`

这段是故意留一个“边界条件”。

好的报告一般不会把结果写成“全解决了”。  
当前 draft 选择承认：

- planar accuracy 仍然是主要剩余问题

这会让全文显得更可信。

---

## 4.9 `Conclusion` 的任务不是加新信息，而是收束 claim

对应正文：

- [`proj2_report_draft.md:405`](./proj2_report_draft.md#L405)
- [`proj2_report_draft.md:422`](./proj2_report_draft.md#L422)

这节在逻辑上做两件事：

1. 把最终保留的 estimator 改动重新列出来
2. 把整篇报告的 claim 范围收紧到一个合理、可 defend 的程度

注意这里的写法是很克制的。  
它没有说：

- 我们拿到了最优 estimator
- 我们解决了所有问题

而是说：

- 我们保留了少量针对性改进
- 它们有实验支持
- 最终完整 mission 能跑完
- 剩余问题主要在 planar 方向

这就是当前 draft 的 claim 边界。

---

## 5. 当前 draft 的“隐藏叙事顺序”

如果把全文更抽象地看，它的内在顺序其实是：

1. 我只讨论 estimator，不抢队友贡献。
2. baseline 的问题是具体的，不是笼统“效果不好”。
3. 所以每个改动都必须能对回某个具体失败模式。
4. 不是所有改动都保留，有些被实验否决了。
5. 最终参数不是按单次最好，而是按更稳、更合理的证据链定稿。
6. 最后把 estimator 放回完整任务里，证明它能支撑整套系统直到 landing 结束。

如果你想修改逻辑，本质上就是在改这六步中的某一步权重。

---

## 6. 如果你想改报告，最常见会改哪几种逻辑

下面是“你可能会提出的修改方向”，以及它们分别对应改哪里。

## 6.1 如果你觉得“现在太像 engineering note，不够像正式 report”

建议改：

- [`proj2_report_draft.md:13`](./proj2_report_draft.md#L13) 到 [`proj2_report_draft.md:15`](./proj2_report_draft.md#L15) 的摘要语气
- [`proj2_report_draft.md:387`](./proj2_report_draft.md#L387) 到 [`proj2_report_draft.md:422`](./proj2_report_draft.md#L422) 的 discussion / conclusion 语气

可做的改法：

- 句子更短
- 更少口语化表达
- 更像“technical report”而不是“调参复盘”

---

## 6.2 如果你觉得“现在虽然清楚，但 failure analysis 太短”

建议改：

- [`proj2_report_draft.md:52`](./proj2_report_draft.md#L52) 到 [`proj2_report_draft.md:73`](./proj2_report_draft.md#L73)

可做的改法：

- 增加早期典型 failure 现象描述
- 补 1 张早期 `z` 轴或平面 lag 的图
- 更明确地区分“z 问题”和“xy 问题”

这是一个非常合理的增强方向，因为老师通常喜欢看到“为什么你知道这是问题”。

---

## 6.3 如果你觉得“设计部分太分散，想更像 textbook / algorithm section”

建议改：

- [`proj2_report_draft.md:75`](./proj2_report_draft.md#L75) 到 [`proj2_report_draft.md:194`](./proj2_report_draft.md#L194)

可做的改法：

- 先统一给完整 state / process / measurement notation
- 再分小节讲 sonar / baro / gps velocity / forward compensation

当前版本是问题驱动写法。  
如果你更想要“算法章节”的感觉，可以把这段改得更系统化。

---

## 6.4 如果你觉得“实验很多，但顺序想调整”

建议改：

- [`proj2_report_draft.md:242`](./proj2_report_draft.md#L242) 到 [`proj2_report_draft.md:331`](./proj2_report_draft.md#L331)

当前顺序是：

- 保留什么
- 删除什么
- 参数怎么定
- 为什么稳定性重要

你也可以改成：

- `z` 轴实验链
- `xy` 实验链
- 最终参数定稿

也就是说，把它从“feature decision order”改成“axis-based order”。

这会让结构更像：

- 先讲 `z`
- 再讲 `xy`

如果你更喜欢这种叙事，我可以按这个方向重写。

---

## 6.5 如果你觉得“最终 run 这一节太像在讲整个任务，不够 estimator-focused”

建议改：

- [`proj2_report_draft.md:333`](./proj2_report_draft.md#L333) 到 [`proj2_report_draft.md:383`](./proj2_report_draft.md#L383)

当前写法虽然已经尽量 estimator 化了，但它仍然会提到：

- landing
- mission completion
- full trajectory

如果你想更 estimator-only，可以改成：

- 只保留“完整 run 中 estimator 误差表现”
- 少写 mission 叙述
- 把 `drone_plan.csv` 仅当“时间对齐辅助证据”

也就是把这节从“full-mission validation”改成“long-horizon estimator validation”。

---

## 6.6 如果你觉得“现在对 `gui_04` 太保守，想更强调它是最终 baseline”

建议改：

- [`proj2_report_draft.md:335`](./proj2_report_draft.md#L335) 到 [`proj2_report_draft.md:339`](./proj2_report_draft.md#L339)

当前写法是比较克制的，因为我主动写了：

- `gui_04` 不能和 `gui_03` 直接一数字比较

这是为了避免 claim 太冒进。

如果你想更强调 `gui_04`，可以改成：

- 它是“最终完整证据”
- `gui_03` 作为“中间确认 run”

但我仍然建议保留“统计窗口不同”的免责声明，因为这是比较诚实、也更容易 defend 的写法。

---

## 6.7 如果你觉得“想更突出 `z` 轴是怎么被解决的”

建议改：

- [`proj2_report_draft.md:52`](./proj2_report_draft.md#L52) 到 [`proj2_report_draft.md:69`](./proj2_report_draft.md#L69)
- [`proj2_report_draft.md:146`](./proj2_report_draft.md#L146) 到 [`proj2_report_draft.md:162`](./proj2_report_draft.md#L162)
- [`proj2_report_draft.md:351`](./proj2_report_draft.md#L351) 到 [`proj2_report_draft.md:371`](./proj2_report_draft.md#L371)

你可以把全文改成更明显的双主线：

- `z` 轴：sensor modeling 修复
- `xy`：lag 修复

这是一个很自然的增强方向，而且和老师喜欢的“问题 -> 解决”风格很匹配。

---

## 7. 当前 draft 最不建议删掉的逻辑点

如果你要压缩篇幅，下面这些逻辑点最好别删：

### 7.1 “只写 estimator”的范围说明

因为这能防止报告和实际分工冲突。

### 7.2 “早期失败模式”这一层

因为没有这一层，后面的设计改动就会像平地起高楼。

### 7.3 “失败尝试也写出来”的逻辑

因为这正是老师邮件里明确鼓励的。

### 7.4 “repeatability matters”

因为这是整篇最像成熟实验设计的一部分。

### 7.5 “`gui_04` 统计窗口和 `gui_03` 不同”的免责声明

因为这是在保护你的论证可信度。

---

## 8. 如果你要让我继续改，最有效的提法是什么

你下一步如果想让我改，最有效的说法不是：

- “这里怪怪的”
- “这个逻辑不太顺”

而是直接说下面这种：

### 8.1 改整篇重心

- “我想把全文改成按 `z` 轴 / `xy` 两条线来写，不要按 feature 来写。”
- “我想让全文更学术一点，不要这么像工程复盘。”
- “我想让全文更像 estimator assignment report，少讲完整 mission。”

### 8.2 改某一节功能

- “failure analysis 太短，我想更明显地写出早期现象。”
- “design section 我想先统一讲数学模型，再讲具体改进。”
- “results section 我想先讲 `z` 轴实验，再讲 `xy` 轴实验。”

### 8.3 改 claim 强度

- “我想把 `gui_04` 写得更强势一点。”
- “我想更保守一点，少说 full mission，只说 long-horizon validation。”
- “我想更强调 `z` 已经解决，`xy` 只是部分缓解。”

---

## 9. 一句话总结当前 draft 的逻辑风格

如果只用一句话概括当前 `proj2_report_draft.md` 的写法，那就是：

> 它不是在写“我实现了哪些 estimator 功能”，而是在写“baseline 出现了哪些可观察、可解释的问题，我们如何用少量但针对性的建模改进和实验筛选，把 estimator 收敛到一个完整任务可用的最终版本”。

如果你认同这个大方向，我们后续就只需要调整每一节的轻重和细节。  
如果你不认同这个大方向，那我也可以直接按你想要的新主线重写正文。

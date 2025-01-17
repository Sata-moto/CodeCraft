# 核心思想：贪心

## 贪心假设：

假设并保证整个生产过程中只存在不超过两层生产的情况（放入材料并且正在生产，或放入材料并且有产品），不会使用三层生产（放入材料，并且正在生产，并且有产品）。三层生产的流水线很难达到（甚至会带来亏损），一般二层生产即可。

我们要求小车只会“顺路”拿取产品，即小车不会空转到某个工作台并拿走它的产品（除开 1/2/3 号工作台或者 stop_frame）。

这意味着，如果材料放入后，上一轮正在生产，则等待至生产完毕（会带来等待误差，该误差由 fun2/fun5 均衡）。

这种假设带来的另一种误差是：只有在开启下一次生产时，才会拿取上一次的产品，那么产品和生产和拿取就被绑定了。

一方面，这会导致 7 号比直接送晚拿到物品从而晚开始。

流水线一旦运行起来，这个就不影响最后的效果了，但是它会延迟流水线运行起来的时间。

另一方面，它还会导致参数的错乱，因为一个开始生产后，即时产品没有拿走，也会有减权。就是说，这个 fun1 的减权影响到了已经生产好的物品。所以 fun2 要去平衡这个影响。

## 决策架构：

### 第一层决策：完成 4/5/6 的任务。

枚举场上所有的 4/5/6，每个 4/5/6 有两个子任务，分别是生产 4-1/4-2 等等，有的子任务已经完成 (input)，有的子任务正在被进行 (occupied) ，有的子任务没有被占用。将所有没有被占用的任务拉取到任务总池中，然后找到任务总池中权重(weight = earning / dis * fun*) 最大的那一个，并分配给小车。标记占用 (occupied)。

占用在一种情况下可以被忽略 (ignore_occupied)。当上一个小车进行了送某个工作台第二个物品的决策时，ignore_occupied 变为 -1，代表当前可以再送一个该种物品（因为上一个物品被送过去时会直接开启生产，并空出格子），如果小车 ignore_occupied，那么 ignore_occupied 变为 1，代表之后不能再进行 ignore，当小车进入检查时，刷新 ignore_occupied。

### 第二层决策：完成送 4/5/6 到 7/9 的任务。

第一层决策在送材料到 4/5/6 完成时，会触发一个 check。

触发 check 时，会连续进行判定。

+ 如果工作台上有物品了，那么小车会先检索场上是否有空闲的 7/9 工作台，
  * 如果没有，
    - 如果当前送的是该工作台的第一个物品，那么小车送到物品，离开并进行新的第一层决策。
    - 如果不是，那么当前工作台阻塞，小车要等待至下一个空闲的 7 号工作台(wait_until_spare_7)，此时整个工作台被占用 (occupied[][0])，并且阻止小车做生产该种物品的决策 (is_waiting_for_7)。
  * 如果有，将产品直接送走，进入第二层决策。
+ 如果工作台上没有物品，那么小车需要观察工作台是否在生产。
  * 如果没有，送到物品，离开并进行新的第一层决策。
  * 如果有，
    - 如果小车送的是这个工作台的第一个物品，送到物品，离开并进行新的第一层决策。
    - 否则，进入等待(wait)，直到该物品生产好。
    

注：触发任何种类的 wait 时，小车停留在当前工作台，不断进行 check，不会做出决策。当小车进入其他决策时，wait 取消。

check 在某些情况下会进入第二层假设，此时小车会查询当前能送到的所有目的地(7/9)，然后选 weight 最大的，如果最大的是 7 号，那么标记占用 occupied。

weight = earning / dis * fun\*。

occupied 忽略效应同上。

小特性：小车等待空闲 7 号时不会提前拿起产品。防止时间损失。

### 第三层决策：完成送 7 到 8/9 的任务

第二层决策在送 4/5/6 到 7 时，触发一个 check。

- 如果此时工作台上有物品：拿走并进入第三层决策。

- 如果工作台上没有物品，而当前正在制作，并且当前是最后一个原料，那么进入等待(wait)。

- 如果当前没有在制作：退回第一层决策。

第三层决策取近即可。

## 控制流特性：

注：make_decision_without_7 是完全独立的一套决策，不要管它。

### map 包：

保存 map 基本信息，包括工作台数量，7/9 工作台数量，和地图。

### desk 包：

保存工作台基本信息，调用 init_desk 获取 total_desk 数组，这个数组将各个种类的工作台分开了。

### car 包：

car 包里面就一个 available_car，它只在 stop_frame 后起作用，代表它是否完成了 stop_frame 之前下发的任务。只有完成了之前的任务的小车才会进行新的决策。

### constant 包：

一些常数和约束关系。

### wait 包：

#### is_waiting_for_7：

某个小车在等待 7 号上的格子时，会触发 is_waiting_for_7[item_type] 标记，代表这个种类的物品制造被阻塞。（这个标记事实上没有使用，因为使用后效果不如不使用）

#### wait：

wait[car_num] 代表当前小车送到了最后一个原料，但是当前产品还没有生产出来，因此进入等待。

#### wait_until_spare_3:

wait_until_spare_3[car_num] 代表当前小车在 1-3 号物品上，并且在等待这些原料的生产。

#### wait_until_spare_7：

wait_until_spare_7[car_num] 同 is_waiting_for_7[item_type]，只是二者标记对象不同。

#### wait_until_spare_sell：

wait_until_spare_sell[car_num] 代表当前小车到达了目的地，想要卖掉物品，但是没办法卖，导致等待。

这种情况出现的唯一原因是：进行了连续的运送（ignore_occupied） 并且上一个运送到后被阻塞（wait）。

#### wait_stop_frame：

这个是 stop_frame 后面的 wait。

#### 拿取策略：

小车会尽量避免 wait，这意味着只要小车送到的不是工作台的最后一种原料（如果是最后一种原料，就必须等待至拿走），小车就不会进入 wait，而是继续做其他决策，

### occupied 包： 

#### occupied:

一般而言，occupied 在任务完成时解除，和工作台的 input_status 一同标识能否进行某个工作台上的决策。

- 对 4-6 号物品，occupied[desk_num][i] 表示 i 号原料正在被某个小车生产。occupied[desk_num][0] 表示当前工作台阻塞，不能再进行新的生产。（阻塞原因是 7 号工作台都满了，即 wait_for_spare_7，wait 不会触发 occupied[][0]，因为它的两个 input 上都有物品）

- 对 7 号物品，occupied[desk_num][i] 表示 i 号原料正在被某个小车生产。

#### sol_occupied:

sol_occupied 的作用是解除 occupied，occpied 不能被手动解除，想解除必须通过 sol_occupied，每一帧开始时会根据 sol_occupied 解除 occupied。

这么做的理由是：occupied 可以在当前帧占用，但是不能在当前帧解除。因为当某个小车放东西到工作台的瞬间，如果解除了 occupied，那么物品实际上只发送了 sell，但是工作台的 input_status 还是 0，所以就会导致这个工作台在这一帧被视为是空闲的。因此必须在下一帧解除。

#### occupied_goods：

用来计算场上的某种物品的数量，包括正在生产的该种物品，运送中的该种物品，工作台上的该种物品，但不包括 7 号工作台上的该种物品。因此计算总数时需要加上七号上的。

#### occupied_stop_frame & sol_occupied_stop_frame:

在 stop_frame 后的 occupied，不与 occupied 混用，但是 occupied 对 stop_frame 后的决策仍有影响。

#### ignore_occupied & sol_ignore_occupied:

ignore_occupied 是用来控制小车做连续的决策的。0 代表不能决策，-1 代表可以决策，1 代表决策过了。

具体而言：当上一个小车完成了某个 4-6 号工作台两个任务的后一个时，ignore_occupied 会由 0 置为 -1。后面的小车做决策时，这个工作台的两个任务虽然都被占用，但是检测到 ignore_occupied，就会允许小车连续的重复运送，并将 ignore_occupied 置 1 防止更多小车进行决策。任意一个小车运送到时，ignore_occupied 置 0。

#### current_occupied：

在进行 check_spare_7 时用到的临时参数，具体在 assist 包种会讲。

### math 包：

可见的是 cddis(car_num,desk_num) 用于计算车和工作台间的距离，dddis(desk_num,desk_num) 用于计算两个工作台间的距离。

### assist 包：

full_6 在小车送到物品时可能触发，判断当前 4-6 号工作台上是否满了（在小车卖掉当前物品后）。

full_7 同上，只是针对的是 7 号工作台。

check_spare_7 在准备送物品到 7 号时触发。用来检索场上有没有空闲的，type 原料格子空着的 7 号工作台。如果有会打上标记（因为在当前帧会有一个小车送过去，而拿个 input_status 不会更新）。

### command 包：

init_dc 是第一帧的初始决策判定，destination 是小车当前的目的地，buy 是小车当前是 buy 还是 sell，check 代表 检查点。 

total_xxx 是小车命令组.

Sel/Buy 在命令组种录入一条指令。

clear_decision 用在 stop_frame 转换的过程中，清除所有原有命令。

md 代表小车需要做出第一层决策。

md_7 代表小车需要做出第二层决策。

md_9 代表小车需要做出第三层决策。

md_stop_frame 代表小车在 stop_frame 之后做出决策。

之所以需要这样子，是因为当前帧认为需要做出决策，但是当前帧的各个状态没有更新，所以真正的决策需要在下一帧做。

## parameter 包：

- seed：seed 为当前种子，在 init 函数中计算，seeds 为各个地图的种子，seed_MOD 为计算种子时使用的哈希模数。

- Stop_frame：当前帧数超过这个值后进入新的决策模式，在新的决策模式下，小车会认为所有工作台不进行生产，决策时只会从一个工作台拿取物品送到另一个工作台。

- Time_Upscale：新模式估计时间时加入的升权参数，防止时间不够

- Earning_Upscale：给大价值物品的升权参数。

- End_frame：认为的截止帧数，防止近的物品 Time 倍数升权不够。

- fun1_desk_exist_num_downscale:给 4,5,6 好工作台上已经有了的物品在 fun1 里的降权。

- dis_pow_downscale：给 4/5/6 号物品运送时路径长度的次方减权，防止路径长度对第二层决策影响过大。

### parameter 列表中的标准参数：

fun1 fun2 fun3 作用于第一层决策（完成哪个 4/5/6 号物品的哪个任务）。

fun4 fun5 fun6 作用于第二层决策（这个物品送到哪去）。

#### fun1（均衡核心参数）:

传入值 remain（场上将要制造的商品的剩余量），该函数会根据 remain 计算权值。计算方式一般是通过一个前端降幅很厉害的降函数进行。目的是让场上的物品尽量平均。是对 “让各个七号工作台都完美的，互不干扰的，不阻塞的运行流水线” 这一目的（后简称【七目】，是决策核心）的粗调，也是效力最强的调整。

其可以做出智能的决策：一方面它可以探究做哪个物品最麻烦，即需要时间最长（所以场上的该种物品会最少），然后让更多的小车去生产该种物品，是流水线的阻塞尽量小从而完成【七目】。另一方面，它和后面的 fun6 配合，可以尽量的让马上就可以生产物品的 7 号工作台先开始生产，从而完成【七目】。

值得注意的是：如果 4/5/6 号物品的第一个任务吃 fun1，但是第二个任务不应该吃 fun1，因为它不会再开启一个工作台的制造线。

#### fun2（核心生产调配参数）：

由于贪心假设【二】（见文档末），使用 fun2 解决误差。方式是：对于刚刚开始生产的物品，有一个减权，从而防止小车等待物品生产时间过长（贪心假设【二】【误差一】）。对于已经生产完毕的物品，有一个加权，从而让小车优先再生产一个该种商品，并顺路卖掉上一个，从而赚取这个已经生产完毕的商品的利润。防止产品不买入并卖出（贪心假设【二】【误差二】）

两个误差当前的加权和减权是对称的，并且根据生产时间线性改变加权。这是不严谨的。可以调整的。

#### fun3（任务连续化参数）：

如果一个任务不是这个工作台发布的首个任务，那么 fun3 就会提供一个加权，从而让一个工作台的各个子任务之间有更好的连续性。

#### fun4（抑制参数）【停止使用】：

### fun5（核心控制参数以及核心出售调配参数）：

该参数实际上可以分为两个参数：一方面它判断目的工作台是不是 7 号，如果不是 7 号（即是 9 号）会提供一个抑制的减权，这个减权会区分两种决策，一种是直接送 9，一种是 【七目】。如果是 7 号就不会改变加权，送入 【七目】 决策中。

另一方面是类似 fun2 的调配参数。效果同 fun2，不赘述。

这个参数有两点与 fun2 不同：

一点是：当某个产品没有开始生产时，它这个生产路径就没有开通。我们为了让各个路径更好的运行，应该更早的开通这条路径，所以应该给一个比较高的加权。

另一点是：如果格子上没到 2 个商品，这个商品加入后，哪怕正在生产，也不会产生等待，所以不需要提供一个减权。

#### fun6（出售调配促进参数）：

fun6 针对没有当前物品的 7 号工作台，它传入 7 号工作台已经放好的原料数量。当原料越多时，越倾向于把当前物品送入那个七号工作台。配合 fun1 尽量的让马上就可以生产物品的 7 号工作台先开始生产，完成 【七目】。

注意一点：fun6 没有考虑当前运送情况，请考虑了历史情况，这意味着如果一个机器人在往某个 7 送 4，另一个机器人准备送 5，但是因为那个 4 还没有送到，所以这个 5 不会优先去送到第一个 7。但是这影不影响【七目】呢？似乎不太影响，因为 fun1 还是会补充两个 7，然后 fun6 起作用。后来也会补上，不是很影响流水线。就是最后几个可能会有一些影响。
注释掉 fun6 的第一行可以部份缓解这个问题。


## 特殊情况说明：

一、7 号很多的地图。

初赛没有这种地图。这种地图可能导致不曾预料的错误，但是影响应该不大。

二、7 号 9 号共同存在的地图。

这是一种常见的地图，但是初赛没有。这种情况需要调整 fun5 的 is_7 权重，从而在去 7 号还是 9 号中做出权衡。





可以考虑的优化：MS 模型，提前等待
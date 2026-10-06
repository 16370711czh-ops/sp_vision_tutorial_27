# 项目理解报告

请尽量使用自己的语言回答以下问题。可以引用少量关键代码或伪代码，但不要只粘贴实现。
完成一节后删除该节末尾的待填写标记；本地检查会拒绝仍有未完成章节的报告。

## 1. 图像生命周期与所有权

ImageSequenceSource 里的 buffer_ 代表相机会反复使用的那块内部缓冲区。
每次读取新图片时，imread 的结果都会覆盖到同一个 buffer_ 上。

cv::Mat 的普通赋值只复制头部信息（尺寸、类型、data 指针等），
底层像素数据仍然由两边共享。所以原来直接写 frame.image = buffer_ 时，
只要下一次 next() 又往 buffer_ 里写数据，已经在队列里的旧 Frame
也会跟着改变，checksum 就对不上了。

我把它改成 frame.image = buffer_.clone()，让每个 Frame 拿到一份独立的像素副本。
这样 Frame 入队之后就完全拥有自己的图像内容，后续复用 buffer_ 不会再影响它。

## 2. 并发处理与恰好一次

producer 每生成一帧就调用 queue_.push 入队，多个 worker 同时调用 queue_.pop。
push 和 pop 内部都有 mutex 保护，并且 pop 会把取出的元素从队列里删除，
所以同一个 Frame 只会被一个 worker 取走，不会重复处理。

producer 读完所有输入后调用 queue_.close()。close 会唤醒所有在 pop 上等待的
worker；如果队列里还有没处理完的帧，worker 会继续取出来处理；
只有当队列已经空了，pop 才会返回 false，worker 才退出循环。

因此已入队的帧不会因为 close 丢掉，也不会被两个 worker 重复处理。

## 3. 共享统计数据

producer 线程会更新 produced，多个 worker 会更新 processed、saved 和 corrupted，
main 线程在最后通过 snapshot() 读取全部计数。

原来的计数是几个普通 int：两个线程可能同时读到同一个旧值，再各自加一写回，
后面的写入会覆盖前面的，导致计数偏小，这就是数据竞争。

我在 Statistics 里加了一个 mutex。四个计数函数在修改计数前都用
std::lock_guard 加锁，离开作用域自动解锁；snapshot() 读取四个计数时也加同一把锁，
所以拿到的快照是一致的一组值，不会读到写了一半的状态。

## 4. 线程关闭协议

第一种情况：调用 start() 后再调用 wait()。
wait() 先 join producer 线程；producer 读完输入后调用 queue_.close()，
worker 处理完剩余帧后退出，wait() 接着 join 所有 worker。
所以 wait() 返回时已经没有任何线程在运行。

第二种情况：调用 start() 后不调用 wait()，直接让 Pipeline 析构。
析构函数里调用 wait()，过程和上面完全一样，因此不会出现
std::thread 仍可 join 就被销毁导致的 std::terminate，
也不会有线程访问已经销毁的对象。

wait() 可以重复调用：每次 join 前都会检查 joinable()，
已经 join 过的线程不会被再次 join，所以先显式 wait() 再析构也是安全的。

这里约定 start() 对同一个 Pipeline 只调用一次。

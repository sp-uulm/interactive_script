#ifndef ASYNC_H
#define ASYNC_H

#include <thread>
#include <memory>
#include <atomic>

class Async {
public:
    using cancel_t = std::shared_ptr<std::atomic<bool>>;

    template<typename Func, typename... Args>
    Async(Func&& f, Args&&... args) {
        cancelled->store(false);
        t = std::thread(std::forward<Func>(f), cancelled, std::forward<Args>(args)...);
    }

    Async() {
    }

    ~Async() {
        cancel();
    }

    Async& operator=(Async&& other) {
        cancel();
        std::swap(t, other.t);
        std::swap(cancelled, other.cancelled);
        return *this;
    }

    void cancel() {
        cancelled->store(true);
        if (t.joinable())
            t.detach();
    }

private:
    std::thread t;
    cancel_t cancelled = std::make_shared<std::atomic<bool>>(false);
};

#endif // ASYNC_H

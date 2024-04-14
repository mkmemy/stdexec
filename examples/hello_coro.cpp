#include <exec/static_thread_pool.hpp>

#include <iostream>
#include <chrono>
#include <stdexec/execution.hpp>
#include <vector>

using namespace stdexec;

auto create(auto begin)
{
    auto first = then(
        begin,
        [&](int v) {
            return v + 1;
        });
    auto second = then(
        first,
        [](int v) {
            return v * 100;
        });
    return second;
}

int main()
{
    exec::static_thread_pool ctx{ 8 };
    auto sch = ctx.get_scheduler();
    auto start = then(schedule(sch), []() { return 999; });
	auto start_time = std::chrono::high_resolution_clock::now();
    auto [value] = sync_wait(create(start)).value();
	auto stop_time = std::chrono::high_resolution_clock::now();
	auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time).count();
    std::cout << value << "\nTook: " << elapsed_time << std::endl;
    return 0;
}
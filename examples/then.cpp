/*
 * Copyright (c) 2021-2022 NVIDIA Corporation
 *
 * Licensed under the Apache License Version 2.0 with LLVM Exceptions
 * (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *
 *   https://llvm.org/LICENSE.txt
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Pull in the reference implementation of P2300:
#include <stdexec/execution.hpp>
#include "./algorithms/then.hpp"

#include <stdexec/execution.hpp>

#include "exec/static_thread_pool.hpp"

#include <cstdio>


///////////////////////////////////////////////////////////////////////////////
// Example code:
int main() {
  auto x = then(stdexec::just(42), [](int i) {
    std::printf("Got: %d\n", i);
    return i;
  });

  // prints:
  //   Got: 42
  [[maybe_unused]] auto [a] = stdexec::sync_wait(std::move(x)).value();
//   (void) a;
    int const nthreads = 4;

    printf("threads: %d\n", nthreads);

    exec::static_thread_pool pool (nthreads);

    auto task = stdexec::schedule(pool.get_scheduler()) 
              | stdexec::bulk(nthreads, [](auto id){ 
                    printf("hello from thread %d\n",id);
                })
				| stdexec::bulk(nthreads, [](auto id) {
					printf("wow...\n");
				});

    stdexec::sync_wait(std::move(task)).value();
}

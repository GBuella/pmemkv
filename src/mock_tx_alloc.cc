/*
 * Copyright 2017, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <libpmemobj/tx_base.h>
#include <dlfcn.h>
#include <cstdlib>
#include <string>
#include <type_traits>
#include <utility>

#include "mock_tx_alloc.h"

thread_local bool tx_alloc_should_fail;

namespace {

template<typename func>
class next_symbol {
    func *address;

public:

    next_symbol(const std::string& name) {
        address = (func*)(dlsym(RTLD_NEXT, name.c_str()));
        if (address == nullptr)
            abort();
    }

    template<typename... Args>
    auto operator()(Args&&... args) -> decltype(address(std::forward<Args>(args)...)) {
        return address(std::forward<Args>(args)...);
    }

};

}

extern "C" PMEMoid pmemobj_tx_alloc(size_t size, uint64_t type_num);

PMEMoid pmemobj_tx_alloc(size_t size, uint64_t type_num) {
    static next_symbol<decltype(pmemobj_tx_alloc)> real{std::string("pmemobj_tx_alloc")};

    if (tx_alloc_should_fail)
        return OID_NULL;
    
    int x;

    return real(size, type_num);
}

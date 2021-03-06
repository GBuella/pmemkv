# pmemkv
Key/Value Datastore for Persistent Memory

*This is experimental pre-release software and should not be used in
production systems. APIs and file formats may change at any time without
preserving backwards compatibility. All known issues and limitations
are logged as GitHub issues.*

Overview
--------

`pmemkv` is a local/embedded key-value datastore optimized for persistent memory.
Rather than being tied to a single language or backing implementation, `pmemkv`
provides different options for storage engines and language bindings.

<ul>
<li><a href="https://github.com/pmem/pmemkv/blob/master/INSTALLING.md">Installation</a>
<ul>
<li><a href="https://github.com/pmem/pmemkv/blob/master/INSTALLING.md#fedora_stable_nvml">Installing on Fedora (Stable NVML)</a>
<li><a href="https://github.com/pmem/pmemkv/blob/master/INSTALLING.md#fedora_latest_nvml">Installing on Fedora (Latest NVML)</a>
<li><a href="https://github.com/pmem/pmemkv/blob/master/INSTALLING.md#building_from_sources">Building From Sources</a>
</ul>
</li>
<li><a href="#engines">Storage Engines</a></li>
<li><a href="#bindings">Language Bindings</a></li>
<li><a href="#benchmarking">Benchmarking</a></li>
</ul>

<a name="engines"></a>

Storage Engines
---------------

`pmemkv` provides multiple storage engines with vastly different implementations. Since all
engines conform to the same common API, any engine can be used with common `pmemkv` utilities
and language bindings. Engines are requested at runtime by name.
[Contributing a new engine](https://github.com/pmem/pmemkv/blob/master/CONTRIBUTING.md#engines)
is easy and encouraged!

![pmemkv-engines](https://user-images.githubusercontent.com/913363/30487989-49e80e86-99f2-11e7-8836-50f27b2ac11a.png)

### Available Engines

| Engine  | Description | Thread-Safe? |
| ------- | ----------- | ------------ | 
| [kvtree](https://github.com/pmem/pmemkv/blob/master/ENGINES.md#kvtree) | Hybrid B+ persistent tree | No |
| [blackhole](https://github.com/pmem/pmemkv/blob/master/ENGINES.md#blackhole) | Accepts everything, returns nothing | Yes |

<a name="bindings"></a>

Language Bindings
-----------------

`pmemkv` is written in C and C++. Developers can either use native C++ classes directly, or use our `extern "C"` API, or use one of several high-level language bindings that are based on the `extern "C"` API.

![pmemkv-bindings](https://user-images.githubusercontent.com/913363/30488014-6221b11e-99f2-11e7-85af-202ad6013e0a.png)

### C++

```cpp
#include <libpmemkv.h>
#include <string.h>
#include <assert.h>
using namespace pmemkv;

int main() {
    // open the datastore
    KVEngine* kv = new KVEngine("kvtree", "/dev/shm/mykv", 8388608);  // 8 MB

    // put new key
    KVStatus s = kv->Put("key1", "value1");
    assert(s == OK);

    // read last key back
    string value;
    s = kv->Get("key1", &value);
    assert(s == OK && value == "value1");

    // close the datastore
    delete kv;

    return 0;
}
```

### C

```c
#include <libpmemkv.h>
#include <string.h>
#include <assert.h>

int main() {
    #define VAL_LEN 64
    char value[VAL_LEN];

    /* open the datastore */
    KVEngine* kv = kvengine_open("kvtree", "/dev/shm/mykv", 8388608);

    /* put new key */
    int32_t len = strlen("value1");
    KVStatus s = kvengine_put(kv, "key1", "value1", &len);
    assert(s == OK);

    /* read last key back */
    len = VAL_LEN;
    s = kvengine_get(kv, "key1", VAL_LEN, value, &len);
    assert(s == OK && !strcmp(value, "value1"));

    /* close the datastore */
    kvengine_close(kv);

    return 0;
}
```

### Other Languages

These bindings are maintained in separate GitHub repos, but are still kept
in sync with the main `pmemkv` distribution.
 
* Java - https://github.com/pmem/pmemkv-java
* JNI - https://github.com/pmem/pmemkv-jni
* Node.js - https://github.com/pmem/pmemkv-nodejs
* Ruby - https://github.com/pmem/pmemkv-ruby

<a name="benchmarking"></a>

Benchmarking
------------

The `pmemkv_stress` utility provides some simple read & write benchmarks.

```
Usage: pmemkv_stress [engine] [command] [value-length] [path] [size]
  engine=kvtree|blackhole
  command=a|r|w|gr|gs|pr|ps|rr|rs
  value-length=<positive integer>
  path=DAX device|filesystem DAX mount|pool set
  size=pool size in MB|0 for device DAX
```  

Benchmarking on emulated persistent memory:

```
cd pmemkv/bin
rm -rf /dev/shm/pmemkv
PMEM_IS_PMEM_FORCE=1 ./pmemkv_stress kvtree a 800 /dev/shm/pmemkv 1000
rm -rf /dev/shm/pmemkv
```

Benchmarking on filesystem DAX:

```
(assuming filesystem mounted at /dev/pmem1)

cd pmemkv/bin
rm -rf /mnt/pmem/pmemkv
PMEM_IS_PMEM_FORCE=1 ./pmemkv_stress kvtree a 800 /mnt/pmem/pmemkv 1000
rm -rf /mnt/pmem/pmemkv
```

Benchmarking on device DAX:

```
(assuming device present at /dev/dax1.0)

cd pmemkv/bin
./pmemkv_stress kvtree a 800 /dev/dax1.0 1000
```

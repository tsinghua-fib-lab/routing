# Routing

### Run

```bash
./routing -h
  -benchmark
        benchmark mode
  -benchmark.aoi_id_end int
        the end aoi id for benchmark (default 500010000)
  -benchmark.aoi_id_start int
        the start aoi id for benchmark (default 500000000)
  -benchmark.count int
        the random routing count for benchmark (default 1000)
  -benchmark.cpu int
        the cpu count for benchmark (default 1)
  -benchmark.seed int
        the seed for benchmark
  -bus string
        bus line database and collection, can be empty [format: {fspath} or {db}.{col}]
  -cache string
        input cache dir path (empty means disable cache)
  -listen string
        gRPC listening address (default "localhost:52101")
  -log-level string
        log level [debug, info, warn, error, fatal, panic] (default "info")
  -map string
        map database and collection [format: {fspath} or {db}.{col}]
  -mongo_uri string
        mongo db uri
  -pprof string
        pprof listening address (default "localhost:52102")
  -road-statuses string
        roadstatuses database and collection, can be empty [format: {fspath} or {db}.{col}]
```

Simplest way to run:

```bash
./routing -map ./map.pb
```

## Attention for Windows Users

Windows Defender may block the execution of the program. You can unblock it by following the steps below:
1. Open Windows Security
2. Click on Virus & threat protection
3. Click on Protection history
4. Click on the "Allow" button for the blocked program

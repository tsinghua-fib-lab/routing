package main

import (
	"errors"
	"flag"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"connectrpc.com/grpcreflect"
	"git.fiblab.net/sim/protos/v2/go/city/routing/v2/routingv2connect"
	easy "git.fiblab.net/utils/logrus-easy-formatter"
	"github.com/sirupsen/logrus"
	"golang.org/x/net/http2"
	"golang.org/x/net/http2/h2c"
)

var (
	// 配置信息
	mongoURI          = flag.String("mongo_uri", "", "mongo db uri")
	mapPathStr        = flag.String("map", "", "map database and collection [format: {fspath} or {db}.{col}]")
	roadStatusPathStr = flag.String("road-statuses", "", "roadstatuses database and collection, can be empty [format: {fspath} or {db}.{col}]")
	cacheDir          = flag.String("cache", "", "input cache dir path (empty means disable cache)")
	grpcEndpoint      = flag.String("listen", "localhost:52101", "gRPC listening address")
	logLevel          = flag.String("log-level", "info", "log level [debug, info, warn, error, fatal, panic]")

	// 性能测试
	benchmark = flag.Bool("benchmark", false, "benchmark mode")
	pprofAddr = flag.String("pprof", "localhost:52102", "pprof listening address")

	LOG_LEVELS = map[string]logrus.Level{
		"debug": logrus.DebugLevel,
		"info":  logrus.InfoLevel,
		"warn":  logrus.WarnLevel,
		"error": logrus.ErrorLevel,
		"fatal": logrus.FatalLevel,
		"panic": logrus.PanicLevel,
	}
)

func main() {
	logrus.SetFormatter(&easy.Formatter{
		TimestampFormat: "2006-01-02 15:04:05.0000",
		LogFormat:       "[%module%] [%time%] [%lvl%] %msg%\n",
	})
	flag.Parse()
	if level, ok := LOG_LEVELS[*logLevel]; ok {
		logrus.SetLevel(level)
	} else {
		logrus.Fatalf("invalid log level: %s", *logLevel)
	}

	mapPath, err := NewPath(*mapPathStr)
	if err != nil {
		logrus.Fatalf("invalid map path: %s", err)
	}
	roadStatusPath, err := NewPath(*roadStatusPathStr)
	if err != nil {
		logrus.Fatalf("invalid road status path: %s", err)
	}
	// 启动导航服务
	server := NewRoutingServer(
		*mongoURI,
		mapPath, roadStatusPath,
		*cacheDir,
	)

	if *pprofAddr != "" {
		// 启动pprof
		startHTTPDebugger(*pprofAddr)
	}

	if *benchmark {
		// 性能测试
		runBenchmark(server)
		return
	}

	// 启动tcp监听和初始化connect服务端
	mux := http.NewServeMux()
	mux.Handle(routingv2connect.NewRoutingServiceHandler(server))
	// 接口反射（可选，主要支持Postman调试）
	reflector := grpcreflect.NewStaticReflector(
		routingv2connect.RoutingServiceName,
	)
	mux.Handle(grpcreflect.NewHandlerV1(reflector))
	mux.Handle(grpcreflect.NewHandlerV1Alpha(reflector))

	addr := *grpcEndpoint
	// port := addr[strings.LastIndex(addr, ":")+1:]
	// 使用HTTP/2 w.o. TLS
	s := &http.Server{
		Addr:    addr,
		Handler: h2c.NewHandler(mux, &http2.Server{}),
	}

	// 优雅退出
	// 创建监听退出chan
	signalCh := make(chan os.Signal, 1)
	//监听指定信号 ctrl+c kill
	signal.Notify(signalCh, syscall.SIGINT, syscall.SIGTERM)
	go func() {
		<-signalCh
		log.Info("stopping...")
		go func() {
			<-signalCh
			os.Exit(1) // 强制结束
		}()
		// 退出connect-go
		s.Close()
		// 退出导航服务
		server.Close()
		os.Exit(0)
	}()

	// 启动gRPC server
	log.Infof("server listening at %v", s.Addr)
	if err := s.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
		log.Fatalf("failed to serve: %v", err)
	}
	time.Sleep(1 * time.Second) // 延迟等待"优雅退出"
	log.Info("routing closes")
}

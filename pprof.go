package main

import (
	"net/http"
	"net/http/pprof"
)

// 访问/debug/pprof/进入pprof实时分析页面
func startHTTPDebugger(addr string) {
	pprofHandler := http.NewServeMux()
	pprofHandler.Handle("/debug/pprof/", http.HandlerFunc(pprof.Index))
	pprofHandler.Handle("/debug/pprof/profile", http.HandlerFunc(pprof.Profile))
	server := &http.Server{Addr: addr, Handler: pprofHandler}
	go server.ListenAndServe()
}

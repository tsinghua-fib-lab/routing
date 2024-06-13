package main

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
)

type Path struct {
	File string
	DB   string
	Coll string
}

func NewPath(filePathOrColl string) (*Path, error) {
	// 检查filePathOrColl是否作为文件存在
	if _, err := os.Stat(filePathOrColl); err == nil {
		return &Path{
			File: filePathOrColl,
		}, nil
	}
	dbDotColl := strings.TrimSpace(filePathOrColl)
	if dbDotColl == "" {
		return nil, nil
	}
	splitted := strings.Split(dbDotColl, ".")
	if len(splitted) != 2 {
		return nil, fmt.Errorf("dbDotColl is invalid: %s", dbDotColl)
	}
	return &Path{
		DB:   splitted[0],
		Coll: splitted[1],
	}, nil
}

func (p *Path) GetDb() string {
	return p.DB
}

func (p *Path) GetColl() string {
	return p.Coll
}

func (p *Path) GetCachePath() string {
	if p.File != "" {
		// return absolute path
		path, err := filepath.Abs(p.File)
		if err != nil {
			log.Panicf("failed to get absolute path of %s: %v", p.File, err)
		}
		return path
	}
	return p.DB + "." + p.Coll + ".pb"
}

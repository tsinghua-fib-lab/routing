package main

import (
	"fmt"
	"os"
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
		return p.File
	}
	return p.DB + "." + p.Coll + ".pb"
}

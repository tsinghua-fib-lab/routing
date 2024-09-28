package algo_test

import (
	"container/heap"
	"testing"

	"git.fiblab.net/sim/routing/v2/router/algo"
	"github.com/stretchr/testify/assert"
)

func TestPriorityQueue(t *testing.T) {
	pq := make(algo.PriorityQueue, 0)
	pq.Push(&algo.Item{Value: 4, Priority: 4})
	pq.Push(&algo.Item{Value: 2, Priority: 2})
	pq.Push(&algo.Item{Value: 1, Priority: 1})
	pq.Push(&algo.Item{Value: 3, Priority: 3})

	// 建堆
	heap.Init(&pq)

	// 弹出
	item := heap.Pop(&pq).(*algo.Item)
	assert.Equal(t, 1, item.Value)
	assert.Equal(t, 1.0, item.Priority)
	item = heap.Pop(&pq).(*algo.Item)
	assert.Equal(t, 2, item.Value)
	assert.Equal(t, 2.0, item.Priority)
}

func TestPriorityQueueChangePriority(t *testing.T) {
	pq := make(algo.PriorityQueue, 0)
	pq.Push(&algo.Item{Value: 4, Priority: 4})
	pq.Push(&algo.Item{Value: 2, Priority: 2})
	pq.Push(&algo.Item{Value: 1, Priority: 1})
	pq.Push(&algo.Item{Value: 3, Priority: 3})

	// 建堆
	heap.Init(&pq)

	// 修改优先级（将Value==3的优先级改为0）
	for _, item := range pq {
		if item.Value == 3 {
			item.Priority = 0
			heap.Fix(&pq, item.Index)
		}
	}

	// 弹出
	item := heap.Pop(&pq).(*algo.Item)
	assert.Equal(t, 3, item.Value)
	assert.Equal(t, 0.0, item.Priority)

	item = heap.Pop(&pq).(*algo.Item)
	assert.Equal(t, 1, item.Value)
	assert.Equal(t, 1.0, item.Priority)

	item = heap.Pop(&pq).(*algo.Item)
	assert.Equal(t, 2, item.Value)
	assert.Equal(t, 2.0, item.Priority)

	item = heap.Pop(&pq).(*algo.Item)
	assert.Equal(t, 4, item.Value)
	assert.Equal(t, 4.0, item.Priority)

	// 空堆
	assert.Equal(t, 0, pq.Len())
}

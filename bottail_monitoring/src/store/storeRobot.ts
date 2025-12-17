import { defineStore } from 'pinia'
import { ref } from 'vue'
import type { RobotState } from '@/types/robotType'

const useRobotStore = defineStore('robot', () => {
  // 로봇 상태값
  const robotState = ref({
    joint: {
      deg: [0, 0, 0, 0, 0, 0],
      vel: [0, 0, 0, 0, 0, 0],
      effort: [0, 0, 0, 0, 0, 0],
    },
    tcp: {
      pos: [0, 0, 0, 0, 0, 0],
    },
    system: {
      state: '',
    },
    timestamp: 0,
  })
  const setRobotState = (state: RobotState) => {
    robotState.value = state
  }

  return {
    robotState,
    setRobotState,
  }
})

export default useRobotStore

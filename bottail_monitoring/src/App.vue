<template>
  <RouterView />
</template>

<script setup lang="ts">
import { RouterView } from 'vue-router'
import { connectROS, subscribeRobotState } from './composable/useRobot';
import { onMounted } from 'vue';
import { subscribeDB } from './composable/useFirebase';


connectROS()


onMounted(() => {
  subscribeDB()
  subscribeRobotState()
})
</script>

<style scoped lang="scss">

</style>

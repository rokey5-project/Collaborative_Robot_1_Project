<template>
  <RouterView />
</template>

<script setup lang="ts">
import { RouterView } from 'vue-router'
import { connectROS } from './utils/ros2Utils';


connectROS()
</script>

<style scoped lang="scss">

</style>

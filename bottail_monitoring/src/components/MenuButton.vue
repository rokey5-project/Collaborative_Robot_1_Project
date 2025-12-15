<template>
  <div
    class="menu-button"
    :class="{'selected': isSelectedMenu()}"
  >
    <div class="menu-text">
      {{ props.data.name }}
    </div>
  </div>
</template>

<script setup lang="ts">
import useCounterStore from '../store/menuData'
import { storeToRefs } from 'pinia';
import type { MenuDataType } from '@/types/menuDataType'
import palette from '../styles/colors'

const { menuState } = storeToRefs(useCounterStore())
const { purple02 } = palette

const props = defineProps<{
  data: MenuDataType
}>();

const isSelectedMenu = () => {
  if(menuState.value === props.data.category) {
    return true;
  }
  return false
}


</script>

<style scoped lang="css">
.menu-button {
  display: flex;
  justify-content: center;
  align-items: center;
  height: 50px;
  padding: 15px 20px;

  .menu-text {
    color: #fff;
    font-size: 36px;
  }

  &.selected {
    background-color: v-bind(purple02);
    border-radius: 10px;
  }
}
</style>
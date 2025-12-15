<template>
  <div class="order-container">
    <MenuBar />
    <MenuItemList />
  </div>
</template>

<script setup lang="ts">
import MenuBar from '@/containers/MenuBar.vue';
import MenuItemList from '@/containers/MenuItemList.vue';

</script>

<style scoped lang="scss">
.order-container {
  display: flex;
  width: 100vw;
  height: 100vh;
}
</style>
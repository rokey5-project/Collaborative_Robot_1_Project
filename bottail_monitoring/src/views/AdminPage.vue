<template>
  <div class="admin-container">
    <AdminMenuBar/>
    <orderDetailsList v-if="adminMenuState === 'orderDetails'"/>
  </div>
</template>
<script setup lang="ts">
import AdminMenuBar from '@/containers/AdminMenuBar.vue';
import OrderDetailsList from '@/components/OrderDetailsList.vue';
import useMenuStore from '@/store/storeMenuData';
import { storeToRefs } from 'pinia';

const {adminMenuState } = storeToRefs(useMenuStore())


</script>
<style scoped lang="scss">
.admin-container {
  width: 100vw;
  height: 100vh;
  display: flex;
}
</style>
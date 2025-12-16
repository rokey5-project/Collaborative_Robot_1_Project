<template>
  <div class="admin-container">
    <AdminMenuBar/>
    <div>
      데이터
    </div>
  </div>
</template>
<script setup lang="ts">
import AdminMenuBar from '@/containers/AdminMenuBar.vue';
import { getDataBase } from '../utils/firebaseUtils'


getDataBase('robot_status')
</script>
<style scoped lang="scss">
.admin-container {
  width: 100vw;
  height: 100vh;
  display: flex;

  .admin-left-container {
    width: 300px;
  }
}
</style>
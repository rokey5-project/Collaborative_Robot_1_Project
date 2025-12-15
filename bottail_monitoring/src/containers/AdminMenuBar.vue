<template>
  <div class="admin-menu-container">
    <div class="admin-menu-title">
      관리자 모드
    </div>
    <div class="admin-menu-list-container">
      <AdminMenuButton
        v-for="menu in menuList"
        :key="menu.category"
        :data="menu"
        @click="clickAdminMenu(menu.category)"
        />
    </div>
  </div>
</template>
<script setup lang="ts">
import palette from '@/styles/colors';
import AdminMenuButton from '@/components/AdminMenuButton.vue';
import type { AdminMenuType } from '@/types/menuDataType';
import useMenuStore from '@/store/storeMenuData';


const { black01 } = palette
const { setAdminMenuState } = useMenuStore()
const menuList: AdminMenuType[] = [
  {
    title: '재고관리',
    category: 'inventory'
  },
  {
    title: '로봇관리',
    category: 'robot'
  },
]

const clickAdminMenu = (category: string) => {
  setAdminMenuState(category)
}

</script>
<style scoped lang="scss">
.admin-menu-container {
  display: flex;
  flex-direction: column;
  background-color: v-bind(black01);

  .admin-menu-title {
    padding: 50px 30px;
    font-size: 32px;
    font-family: 'NanumGothicEcoBold', sans-serif;
    color: #fff;
  }

  .admin-menu-list-container {
    flex: 1
  }

}

</style>
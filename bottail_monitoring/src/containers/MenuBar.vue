<template>
  <div class="left-menu-bar">
    <div class="img-container">
      <img src="../assets/bottail_logo_text.png" alt="logo-text" class="logo-text"/>
    </div>
    <div class="menu-list-container">
        <MenuButton
          v-for="menu in menuList"
          :key="menu.category"
          :data="menu"
          @click="setMenuState(menu.category)"
        />
    </div>
    <div class="menu-setting-container">
      <div class="menu-setting-icon-wrap">
        <img src="../assets/setting.svg" alt="setting-icon" class="menu-setting-icon"/>
      </div>
      <div class="menu-setting-text" @click="goToAdminPage">
        관리자모드
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import router from '@/router';
import MenuButton from '../components/MenuButton.vue'
import palette from '../styles/colors'
import useCounterStore from '../store/menuData'

const { purple01 } = palette
const { menuList, setMenuState } = useCounterStore()

const goToAdminPage = () => {
  router.push({ name: 'admin' });
}
</script>

<style scoped lang="scss">
  .left-menu-bar {
    width: 20vw;
    background-color: v-bind(purple01);
    height: 100vh;
    border-top-right-radius: 5vw;
    display: flex;
    flex-direction: column;
    justify-content: space-between;

    .img-container {
      display: flex;
      justify-content: center;
      align-items: center;
      padding: 30px 20px 0 20px;

      .logo-text {
        width: 220px;
        height: 100px;
      }
    }

    .menu-list-container {
      display: flex;
      flex-direction: column;
      gap: 20px;
      padding: 20px 0;
    }

    .menu-setting-container {
      display: flex;
      justify-content: center;
      align-items: center;
      background-color: #000;
      height: 70px;
      padding: 10px;
      gap: 10px;

      .menu-setting-icon-wrap {
        width: 37px;
        height: 37px;

        .menu-setting-icon {
          width: 100%;
          height: 100%;
        }
      }

      .menu-setting-text {
        color: #fff;
        font-size: 32px;
        font-family: 'NanumGothicEcoBold', sans-serif;
      }
    }

  }
</style>
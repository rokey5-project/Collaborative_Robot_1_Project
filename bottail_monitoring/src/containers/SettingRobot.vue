<template>
  <div class="setting-robot-container">
    <div class="setting-robot-title">
      로봇 관리
    </div>
    <div class="setting-robot-body">
      <el-button type="danger" @click="stopRobot">
        정지 버튼
      </el-button>
    </div>
  </div>
</template>

<script setup lang="ts">

const stopRobot = () => {
  console.log('정지')
}

</script>

<style scoped lang="scss">
.setting-robot-container {
  flex: 1;
  padding: 30px;
  display: flex;
  flex-direction: column;
  gap: 30px;

  .setting-robot-title {
    font-size: 32px;
    font-family: 'NanumGothicEcoBold', sans-serif;
  }

  .setting-robot-body {
    flex: 1;

  }
}

</style>
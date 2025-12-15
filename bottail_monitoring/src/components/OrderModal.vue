<template>
  <el-dialog
    v-model="modalState"
    title="주문 확인"
  >
    <span>This is a message</span>
    <template #footer>
      <div class="dialog-footer">
        <el-button @click="openCloseModal">
          취소
        </el-button>
        <el-button type="primary" @click="openCloseModal">
          주문하기
        </el-button>
      </div>
    </template>
  </el-dialog>
</template>

<script lang="ts" setup>
import useCounterStore from '@/store/menuData';
import { storeToRefs } from 'pinia';

const { modalState } = storeToRefs(useCounterStore())
const { openCloseModal } = useCounterStore()

</script>

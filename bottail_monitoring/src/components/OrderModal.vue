<template>
  <el-dialog
    v-model="modalState"
    class="modal-container"
  >
    <div class="modal-title">
      주문하시겠습니까??
    </div>
    <div class="modal-item-container">
      <div class="modal-item-img-wrap">
        <img
          :src="orderItemInfo.image"
          :alt="orderItemInfo.category"
          class="modal-item-img"
          />
      </div>
      <div class="modal-item-info">
        <div class="modal-item-name-wrap">
          <div class="modal-item-name-text">
            {{ orderItemInfo.name }}
          </div>
        </div>
        <div class="modal-item-price-wrap">
          <div class="modal-item-price-text">
            {{ getPriceText(orderItemInfo.price) }}
          </div>
        </div>
        <div class="modal-item-desc-wrap">
          <div class="modal-item-desc-text">
            {{ orderItemInfo.description }}
          </div>
        </div>
      </div>
    </div>
    <template #footer>
      <div class="dialog-footer">
        <el-button size="large" type="danger" @click="openCloseModal">
          취소
        </el-button>
        <el-button size="large" type="primary" @click="clickConfirmButton">
          주문하기
        </el-button>
      </div>
    </template>
  </el-dialog>
</template>

<script lang="ts" setup>
import useCounterStore from '@/store/storeMenuData';
import palette from '../styles/colors'
import { storeToRefs } from 'pinia';
import { ElMessage } from 'element-plus'
import { publishOrderTopic } from '../utils/ros2Utils'

const { modalState, orderItemInfo } = storeToRefs(useCounterStore())
const { openCloseModal } = useCounterStore()

const { gray01 } = palette

const getPriceText = (price: number) => {
  return `${price?.toLocaleString()}원`
}

const clickConfirmButton = () => {
  publishOrderTopic('highball')
  ElMessage({
    message: '주문이 완료되었습니다.',
    type: 'success',
  })

  openCloseModal()
}
</script>

<style scoped lang="scss">
  .modal-container {

    .modal-title {
      font-size: 28px;
      margin-left: 20px;
    }

    .modal-item-container {
      display: flex;
      gap: 30px;
      padding: 30px 30px 0 30px;

      .modal-item-img-wrap {
        width: 200px;
        height: 200px;
        display: flex;
        justify-content: center;
        align-items: center;

        .modal-item-img {
          width: 100%;
          height: 100%;
          object-fit: fill;
          border-radius: 10px;
        }
      }

      .modal-item-info {
        flex: 1;
        display: flex;
        flex-direction: column;
        gap: 10px;
      }

      .modal-item-name-wrap {
        display: flex;
        align-items: center;
        box-sizing: border-box;

        .modal-item-name-text {
          font-size: 36px;
        }
      }

      .modal-item-price-wrap {
        display: flex;
        align-items: center;
        box-sizing: border-box;

        .modal-item-price-text {
          font-size: 28px;
        }
      }

      .modal-item-desc-wrap {
        display: flex;
        align-items: center;
        box-sizing: border-box;

        .modal-item-desc-text {
          margin-top: 10px;
          font-size: 18px;
          color: v-bind(gray01);
        }
      }
    }
  }
</style>
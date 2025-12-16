<template>
  <RouterView />
</template>

<script setup lang="ts">
import { RouterView } from 'vue-router'
import { connectROS } from './utils/ros2Utils';
import { ref, onValue} from 'firebase/database'
import { database } from '@/firebase'
import useMenuStore from './store/storeMenuData';
import { onMounted } from 'vue';
import type { OrderDetailsType } from './types/menuDataType';

const { setOrderDetailsList } = useMenuStore()

const subscribeDB = () => {
  const orderDetailsRef = ref(database, 'orderDetails')

  if (orderDetailsRef) {
    onValue(orderDetailsRef, (snapshot) => {
    const data = snapshot.val()
    const list:OrderDetailsType[] = data ? Object.values(data) : []
    console.log('DB data', list)

    setOrderDetailsList(list)
    })
  }
}

connectROS()


onMounted(() => {
  subscribeDB()
})
</script>

<style scoped lang="scss">

</style>

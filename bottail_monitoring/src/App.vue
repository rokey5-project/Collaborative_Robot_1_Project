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

const { setOrderDetailsList } = useMenuStore()

const orderDetailsRef = ref(database, 'orderDetails')

const subscribeDB = onValue(orderDetailsRef, (snapshot) => {
  const data = snapshot.val()
  console.log('DB data', data)
  setOrderDetailsList(data)
})

connectROS()


onMounted(() => {
  subscribeDB()
})
</script>

<style scoped lang="scss">

</style>

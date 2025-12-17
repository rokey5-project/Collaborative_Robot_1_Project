/* eslint-disable @typescript-eslint/no-explicit-any */
import { ref, onValue, get, set, push, update } from 'firebase/database'
import { database } from '@/firebase'
import type { OrderDetailsType } from '../types/menuDataType'
import useMenuStore from '../store/storeMenuData'

const { setOrderDetailsList } = useMenuStore()

// 주문 내역 저장 (DB 변동이 생길때마다 동작)
const subscribeDB = () => {
  const orderDetailsRef = ref(database, 'orderDetails')

  if (orderDetailsRef) {
    onValue(orderDetailsRef, (snapshot) => {
      const data = snapshot.val()
      const list: OrderDetailsType[] = data ? Object.values(data) : []
      console.log('DB data', list)

      setOrderDetailsList(list)
    })
  }
}

// DB 데이터 가져오기
const getDataBase = async (path: string) => {
  const data = await get(ref(database, path))

  if (data.exists()) {
    console.log('DB data', data.val())
    return data.val()
  } else {
    console.log('데이터가 없습니다.')
  }
}

// DB 데이터 저장
const setDataBase = async (path: string, data: any) => {
  await set(ref(database, path), data)
}

// DB 데이터 추가
const pushDataBase = async (path: string, data: any) => {
  const orderDetailsRef = ref(database, path)

  push(orderDetailsRef, data)
}

// DB 데이터 업데이트
const updateDataBase = (path: string, data: any) => {
  update(ref(database, path), data)
}

export { subscribeDB, getDataBase, setDataBase, pushDataBase, updateDataBase }

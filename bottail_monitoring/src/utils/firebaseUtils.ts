/* eslint-disable @typescript-eslint/no-explicit-any */
import { ref, get, set, update, push } from 'firebase/database'
import { database } from '@/firebase'

//데이터 읽기
export const getDataBase = async (path: string) => {
  const data = await get(ref(database, path))

  if (data.exists()) {
    console.log('DB data', data.val())
    return data.val()
  } else {
    console.log('데이터가 없습니다.')
  }
}

// 데이터 설정
export const setDataBase = async (path: string, data: any) => {
  await set(ref(database, path), data)
}

// 기존 데이터에 데이터 추가
export const pushDataBase = async (path: string, data: any) => {
  const orderDetailsRef = ref(database, path)

  push(orderDetailsRef, data)
}

// 데이터 수정
export const updateDataBase = (path: string, data: any) => {
  update(ref(database, path), data)
}

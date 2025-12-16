/* eslint-disable @typescript-eslint/no-explicit-any */
import { ref, get, set, update } from 'firebase/database'
import { database } from '@/firebase'

export const getDataBase = async (path: string) => {
  const data = await get(ref(database, path))
  console.log(data.val())
}

export const setDataBase = async (path: string, data: any) => {
  await set(ref(database, path), data)
}

export const updateDataBase = (path: string, data: any) => {
  update(ref(database, path), data)
}

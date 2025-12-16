/* eslint-disable @typescript-eslint/no-explicit-any */
import { ref, set, update } from 'firebase/database'
import { database } from '@/firebase'

export const setDataBase = async (path: string, data: any) => {
  await set(ref(database, path), data)
}

export const updateDataBase = (path: string, data: any) => {
  update(ref(database, path), data)
}

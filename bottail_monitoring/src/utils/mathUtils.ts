export const degreeToRadians = (arr: number[]) => {
  return arr.map(angle => (angle * (Math.PI / 180)));
}

export const radiansToDegrees =(arr: number[]) => {
  return arr.map(rad => rad * (180 / Math.PI));
}

export function swapIndices(arr: number[], index1: number, index2: number): number[] {
  if (arr.length <= Math.max(index1, index2)) {
    throw new Error("배열의 길이가 인덱스 범위를 초과합니다.");
  }

  // 값이 확실히 undefined가 아니라고 가정하고 강제 타입 지정
  const temp = arr[index1]!;
  arr[index1] = arr[index2]!;
  arr[index2] = temp;

  return arr;
}
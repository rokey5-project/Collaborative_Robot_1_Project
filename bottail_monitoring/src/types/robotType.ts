export type RobotState = {
  joint: {
    deg: number[]
    vel: number[]
    effort: number[]
  }
  tcp: {
    pos: number[] // [x, y, z, rx, ry, rz]
  }
  system: {
    state: string
  }
}

export type TopicConfig = {
  name: string
  messageType: string
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  handler: (msg: any) => void
}

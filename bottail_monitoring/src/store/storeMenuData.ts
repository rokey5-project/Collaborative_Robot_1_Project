import { ref } from 'vue'
import { defineStore } from 'pinia'
import type { MenuDataType, MenuItemType } from '@/types/menuDataType'

const useMenuStore = defineStore('menu', () => {
  // 메뉴 정보
  const menuList: MenuDataType[] = [
    {
      category: 'whisky',
      name: '위스키 베이스',
      item: [
        {
          category: 'manhattan',
          name: '맨해튼',
          price: 1000,
          description: '위스키와 베르무트의 조화가 돋보이는 클래식 칵테일',
          image:
            'https://i.namu.wiki/i/3ZbFQNaAzR5vFrqJHSwXER32xpOEqX7OecmC_hmr7nmbQOS_7y_3rGPJRRxYgWhKGfOIgFVXdINoANobWUt2ta29ROFi8w2EeHSvr7QGnGaxndiMT3AKNP4LoCs-dGJ8ETzral96jgtfWdSFLrpSbQ.webp',
        },
        {
          category: 'jackCoke',
          name: '잭콕',
          price: 900,
          description: '잭다니엘과 콜라의 부담 없는 하이볼 스타일',
          image:
            'https://i.namu.wiki/i/ELEHsyUTDjPeA68tLYnOIodZhbUVeTu5dbo9az7dRYM-f1V66uDEiA9Z_q3bO1ut9AIRP2XsdelkT4V4ywIjdQztLapTTyLrtcfHMmm5BcT_RV64W_MdPbH0hNQrkcdudueLH6aYn-vbf6kgHucobA.webp',
        },
        {
          category: 'godfather',
          name: '갓파더',
          price: 1000,
          description: '위스키에 아마레또를 더한 달콤하고 묵직한 맛',
          image:
            'https://i.namu.wiki/i/OhDF4mtR2uu5jVivLoEAqpRIBPTZbyPvSJ7eDo8hVLXz5iYzF8nWyFNkkr_afftpXDM_wH-X2qgscC3gO9RwauX9AvLucaexDxC1MeVjAZpWSGh7xUR4LlE-KPDNMw9XTVGBTgWi0fIbT7YbumcZ7Q.webp',
        },
        {
          category: 'oldFashioned',
          name: '올드 패션드',
          price: 1100,
          description: '설탕과 비터로 위스키 본연의 풍미를 살린 정통 칵테일',
          image:
            'https://i.namu.wiki/i/dOSvTuLJZSS3R59SB3-GtLldqvF5WP7RpAzZyabGIiZE2HM8wPbt0iLg2F3K_ZvVewg7XZ30_bWp8NyS9HTBj3eYDAVLakNfhfEGn2_XXpvB8wLKzLCD_DtrCN1XJlJWbSy_5PSZ-kMoCOnArDalEA.webp',
        },
        {
          category: 'highball',
          name: '하이볼',
          price: 800,
          description: '탄산과 위스키의 청량하고 가벼운 조합',
          image:
            'https://i.namu.wiki/i/HwxVTECoEw2AhjtxsPMv-tDbhKgdHH0Z_QJOvWQty7SjUmmF9oTRcY1_umXUh23_UcJf_GXKrr28oUDXYmZZpAKOdJGaOkcrXXEBdkx-8eUVZW86k5F_1kbpeIuKn_zVydzHcXBq6-CYy_Dgu7SSfg.webp',
        },
      ],
    },
    {
      category: 'jin',
      name: '진 베이스',
      item: [
        {
          category: 'martini',
          name: '마티니',
          price: 1100,
          description: '진과 드라이 베르무트의 깔끔한 어른의 맛',
          image:
            'https://i.namu.wiki/i/EKqoKGJ1cwJusexjo3hZEsaDHGhJ87if3uthHqaPQMi21ALhSv_DHEFuoZMUQWdLMheSPn1Iss-9jb7G3rlrGaMNVIewUxGFgKI8axXGy85ZLfO3D2zmZvytlXsHzyuH_u-jGAAvFnZ6o55k72lIlw.webp',
        },
        {
          category: 'ginTonic',
          name: '진토닉',
          price: 900,
          description: '허브 향의 진과 토닉워터의 상쾌한 조합',
          image:
            'https://i.namu.wiki/i/0kXEMSFiviwaQULl1TyXGAqGGIxY1aTQbKZogJOft88iK6lBpK7HOHKjUF0aRyqLcRNKUpgZ5Hqg3W0JWXbkqiyInYT0GTd87CFn8j2p7n-EE-FFigf_UD9mk4vgHVcYQKCHIG2LbsJMETk1ujBAPQ.webp',
        },
        {
          category: 'longIslandIcedTea',
          name: '롱 아일랜드 아이스 티',
          price: 1000,
          description: '여러 술이 어우러진 강렬하지만 의외로 부드러운 칵테일',
          image:
            'https://i.namu.wiki/i/fJ2RwBNJ6nP-me5zrQdKAwdxPz_i4Trla3iJvR9MZRr89Fo9HgFcz7ORWmRO3BQlTiwQykAkLDmUKl6O0AkyYE39lbCCfmig71WUGNDk5kyIsBKqd-ZXF4U_tsGnosfvx7qdy3zkXVTLWYvXJIbxbw.webp',
        },
      ],
    },
    {
      category: 'vodka',
      name: '보드카 베이스',
      item: [
        {
          category: 'moscowMule',
          name: '모스코 뮬',
          price: 800,
          description: '보드카와 진저비어의 상쾌한 스파이시함',
          image:
            'https://i.namu.wiki/i/JygmuplU2oX2vkffWVWKViIZJHkC7EY4pE7M3O3fAaAWByjYlEOWxo-AdltGcg-7AQjfS7LCTjxTrLckMMw6m9HiB-1gM6Ihxy0N3kWf9MGd2CXyS_jUDr734mkK6S_kvYiMnbgfjjsvXd7y1S7djw.webp',
        },
        {
          category: 'blackRussaian',
          name: '블랙 러시안',
          price: 900,
          description: '보드카와 커피 리큐르의 달콤 쌉싸름한 조합',
          image:
            'https://i.namu.wiki/i/B9M9-zpHU-G_q6JzUHj5dogevr_y-vHqBc0rSwFgQLfjijfAS4CghpYeZ5fzMH3b21hxDavnJria3-HiWdof7mYcRWJ8NcUCQ4cyL33W7YRInxbfgSfeUiNLJcf7a_vPghAd0z6Yy6TS5gotgWZBCQ.webp',
        },
        {
          category: 'cosmopolitan',
          name: '코스모폴리탄',
          price: 1100,
          description: '상큼한 크랜베리와 라임이 어우러진 세련된 맛',
          image:
            'https://i.namu.wiki/i/S_Z7yzhB493bIHsxLVPu8InEGo0V8-ZUwcs8aNOzQSOkGAFOqItUa7qpVjqKVrM1u9W3aX9lSMxfPlcK2pOaySwymNrzx1am7lFqzy5czBRx4lj9OwxlnjIGsNgUtkwpglMV8xi0tUIzPedUEnPy7g.webp',
        },
        {
          category: 'buleLagoon',
          name: '블루 라군',
          price: 1000,
          description: '선명한 블루 컬러의 달콤하고 청량한 칵테일',
          image:
            'https://i.namu.wiki/i/B97SSJpathUqXMWhhoMC2nxY7uK2Zi52D_DFOYzMSCTeHPwKbH2xYZCp0BRGZTDHkpUXX2bysuvVhMF3bNen8NDdk2KUJb2yHYv7KFQyKYsNsoCloFq70nDyipMGkuhk4l1vy0HSNtx0V113E7Qp3Q.webp',
        },
        {
          category: 'seaBreeze',
          name: '시 브리즈',
          price: 1100,
          description: '과일 주스가 어우러진 가볍고 산뜻한 맛',
          image:
            'https://i.namu.wiki/i/CH2sGK0V6ebT1GnEDsTMFiDLB_U0ML-sFvGz57aIDFu2qQQ1eLP-FPTOubu9oED8TZo82Ar2AOvzW3oQQjnweafTsN-Uqu8b-kekYVx8nDYMD4WHmSqPvOYPFGwMRCX3eCdNiwAjN4lHJ7qImihMHA.webp',
        },
      ],
    },
    {
      category: 'rum',
      name: '럼 베이스',
      item: [
        {
          category: 'mojito',
          name: '모히또',
          price: 1100,
          description: '민트와 라임이 어우러진 대표적인 상큼 칵테일',
          image:
            'https://i.namu.wiki/i/XN5dMAiR9sao94N-gP7m5k-GwoUDMLoEHDjYqbPhCAGoYPVxQTQTaE29JU6yc8xpkuPvviGaNlPI2tmvDsP0HeBR7VELJx-1ATxcHVfx6t3Svd0sUBwq3fFmwiRXfWY_8PfKLdNj2AYiYvcYdj_hyQ.webp',
        },
        {
          category: 'bacardi',
          name: '바카디',
          price: 1000,
          description: '럼의 깔끔한 풍미를 느낄 수 있는 베이직 칵테일',
          image:
            'https://i.namu.wiki/i/F_B8mYSEcfMdLUrkJ8hEmfX8MyGaNE4hps4HCs1NTT4PHD7LecOfID3FjIKwN5piHBukA-7rxmSqj9-NrkUvYW9UdptHh-8Ye0vJvpyqP6gGqNpXSkcWXazwLJ55yC5gt-NDEsR3Kr9NaCP2Gl4F2A.webp',
        },
        {
          category: 'cubalibre',
          name: '쿠바 리브레',
          price: 900,
          description: '럼과 콜라에 라임을 더한 시원한 맛',
          image:
            'https://i.namu.wiki/i/Y7awz3G19bLEbBCrFAnEiGfNG-JgIhXX4-skaZCdwR7V0fUVRJ_tW4yQAyqEKlw3pBrrgwPFKejIk9HULXQx3KiFMUxdhHxZBVDW5dCqEdy8VvEPXG6eshScR_1P6ccxI2K92z6vDBjqmy4gxVUHjQ.webp',
        },
      ],
    },
    {
      category: 'liqueur',
      name: '리큐르 베이스',
      item: [
        {
          category: 'midoriSour',
          name: '미도리 사워',
          price: 1100,
          description: '멜론 리큐르의 달콤하고 상큼한 맛',
          image:
            'https://i.namu.wiki/i/-9I-WV1sV6EghdDrLY-iXw2lcn383YozTd45QxEdvuyciM8bThZ9jBaycZhsjCDUWoBkSTL67a0HcSOWypc5yXuZEdwVw1kkjIms-UVQLy_J9QyWC_PVC9D2nznpqeu96nfrFoKLEWxit6tPSiS_cA.webp',
        },
        {
          category: 'juneBug',
          name: '준벅',
          price: 900,
          description: '열대과일 풍미의 달콤한 파티 칵테일',
          image:
            'https://i.namu.wiki/i/GqRieyymqhooW8HejohkwzfMfYw_JmUHsb0yMgUajG0ZMH70_w8QQdTko8i5AQX6j8LMGcp9_7M0VGY21glD1F_m882UNC2yE_a3vuNg5rj8FwtuDZTrG8Wyxeo9YGDxoqgZSrJ2qG-NaG4uFt3M3g.webp',
        },
        {
          category: 'greenWidow',
          name: '그린 위도우',
          price: 1000,
          description: '허브 향이 인상적인 독특한 리큐르 칵테일',
          image:
            'https://i.namu.wiki/i/4OHkeJhkHe93JM_HRiPTVk6D7UHHkEyfjxzr45kAmfy1hoMN_I0L_x7SPZ79wIaKBegz9QjOt1Xhu2cYGFpJNg1lwTbeoHXTPspIINHks0pSu6HfRHesJRq4r413KlFPj8US-yRALAb18QajnG2F1A.webp',
        },
        {
          category: 'fuzzyNavel',
          name: '퍼지 네이블',
          price: 1000,
          description: '복숭아 향이 부드럽게 퍼지는 달콤한 칵테일',
          image:
            'https://i.namu.wiki/i/AA90ehCDjZVftIDN9cbCRG17KG3IRfa641d9eR32yXXF6CQIdDvpqjZDLizD3vQvwTN3QbF1nATvXJfmBC9_NU4Ij_Avh9pi0zVj9p9yDjl6BDP0Qz-09R-ZyEoln2f__S_LmA7SG2uOhNaZiGkmkQ.webp',
        },
        {
          category: 'spritz',
          name: '스프릿츠',
          price: 1100,
          description: '가볍고 탄산감 있는 이탈리아식 아페리티프',
          image:
            'https://i.namu.wiki/i/qkheVC0PlqJrwDkDosMnAIriUO1EmIzg9kl8Qf8B-xhC8M-PWNXLA9ooW43xVo76s_4O07u56X6tu8e3HBd6j33cdN8UgSbYB7CNMOnqQpnttil15LDwOrQ_X-S9zj0XTqeFweTnRoOLrrc6K7kvpw.webp',
        },
        {
          category: 'brainHemorrhage',
          name: '브레인 헤머리지',
          price: 1200,
          description: '비주얼이 강렬한 달콤한 샷 칵테일',
          image:
            'https://i.namu.wiki/i/4iHGknkD05-C2dacdGYR71fxYhXHOSj-EkCfmk2ltS4e6_8AxFf7H6VNxz3ljUcKcaG7JlBHlAvJQSKMdaDCRaK91gaERgJpXC21nSoStwtdX0wOTes2ck5POvVeb7tMnMupWInrVltM2A_BF932ww.webp',
        },
      ],
    },
  ]

  // 메뉴판 버튼 클릭 상태
  const menuState = ref('whisky')
  const setMenuState = (category: string) => {
    menuState.value = category
  }

  // 어드민 페이지 버튼 클릭 상태
  const adminMenuState = ref('orderDetails')
  const setAdminMenuState = (category: string) => {
    adminMenuState.value = category
  }

  // 주문하려는 아이템 정보
  const orderItemInfo = ref({} as MenuItemType)
  const setOrderItemInfo = (data: MenuItemType) => {
    orderItemInfo.value = data
  }

  return {
    menuList,
    menuState,
    setMenuState,
    orderItemInfo,
    setOrderItemInfo,
    adminMenuState,
    setAdminMenuState,
  }
})

export default useMenuStore

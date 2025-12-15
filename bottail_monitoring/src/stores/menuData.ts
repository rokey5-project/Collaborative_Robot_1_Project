import { defineStore } from 'pinia'

export const useCounterStore = defineStore('menu', () => {
  const menuList = [
    {
      category: 'wisky',
      name: '위스키 베이스',
      item: [
        {
          category: 'manhattan',
          name: '맨해튼',
          pirce: 1000,
          image:
            'https://i.namu.wiki/i/3ZbFQNaAzR5vFrqJHSwXER32xpOEqX7OecmC_hmr7nmbQOS_7y_3rGPJRRxYgWhKGfOIgFVXdINoANobWUt2ta29ROFi8w2EeHSvr7QGnGaxndiMT3AKNP4LoCs-dGJ8ETzral96jgtfWdSFLrpSbQ.webp',
        },
        {
          category: 'jackCoke',
          name: '잭콕',
          pirce: 900,
          image:
            'https://i.namu.wiki/i/ELEHsyUTDjPeA68tLYnOIodZhbUVeTu5dbo9az7dRYM-f1V66uDEiA9Z_q3bO1ut9AIRP2XsdelkT4V4ywIjdQztLapTTyLrtcfHMmm5BcT_RV64W_MdPbH0hNQrkcdudueLH6aYn-vbf6kgHucobA.webp',
        },
        {
          category: 'godfather',
          name: '갓파더',
          pirce: 1000,
          image:
            'https://i.namu.wiki/i/OhDF4mtR2uu5jVivLoEAqpRIBPTZbyPvSJ7eDo8hVLXz5iYzF8nWyFNkkr_afftpXDM_wH-X2qgscC3gO9RwauX9AvLucaexDxC1MeVjAZpWSGh7xUR4LlE-KPDNMw9XTVGBTgWi0fIbT7YbumcZ7Q.webp',
        },
        {
          category: 'oldFashioned',
          name: '올드 패션드',
          pirce: 1100,
          image:
            'https://i.namu.wiki/i/dOSvTuLJZSS3R59SB3-GtLldqvF5WP7RpAzZyabGIiZE2HM8wPbt0iLg2F3K_ZvVewg7XZ30_bWp8NyS9HTBj3eYDAVLakNfhfEGn2_XXpvB8wLKzLCD_DtrCN1XJlJWbSy_5PSZ-kMoCOnArDalEA.webp',
        },
        {
          category: 'highball',
          name: '하이볼',
          pirce: 800,
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
          pirce: 1100,
          image:
            'https://i.namu.wiki/i/EKqoKGJ1cwJusexjo3hZEsaDHGhJ87if3uthHqaPQMi21ALhSv_DHEFuoZMUQWdLMheSPn1Iss-9jb7G3rlrGaMNVIewUxGFgKI8axXGy85ZLfO3D2zmZvytlXsHzyuH_u-jGAAvFnZ6o55k72lIlw.webp',
        },
        {
          category: 'ginTonic',
          name: '진토닉',
          pirce: 900,
          image:
            'https://i.namu.wiki/i/0kXEMSFiviwaQULl1TyXGAqGGIxY1aTQbKZogJOft88iK6lBpK7HOHKjUF0aRyqLcRNKUpgZ5Hqg3W0JWXbkqiyInYT0GTd87CFn8j2p7n-EE-FFigf_UD9mk4vgHVcYQKCHIG2LbsJMETk1ujBAPQ.webp',
        },
        {
          category: 'ginTonic',
          name: '진토닉',
          pirce: 900,
          image:
            'https://i.namu.wiki/i/0kXEMSFiviwaQULl1TyXGAqGGIxY1aTQbKZogJOft88iK6lBpK7HOHKjUF0aRyqLcRNKUpgZ5Hqg3W0JWXbkqiyInYT0GTd87CFn8j2p7n-EE-FFigf_UD9mk4vgHVcYQKCHIG2LbsJMETk1ujBAPQ.webp',
        },
        {
          category: 'longIslandIcedTea',
          name: '롱 아일랜드 아이스 티',
          pirce: 1000,
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
          pirce: 800,
          image:
            'https://i.namu.wiki/i/JygmuplU2oX2vkffWVWKViIZJHkC7EY4pE7M3O3fAaAWByjYlEOWxo-AdltGcg-7AQjfS7LCTjxTrLckMMw6m9HiB-1gM6Ihxy0N3kWf9MGd2CXyS_jUDr734mkK6S_kvYiMnbgfjjsvXd7y1S7djw.webp',
        },
        {
          category: 'blackRussaian',
          name: '블랙 러시안',
          pirce: 900,
          image:
            'https://i.namu.wiki/i/B9M9-zpHU-G_q6JzUHj5dogevr_y-vHqBc0rSwFgQLfjijfAS4CghpYeZ5fzMH3b21hxDavnJria3-HiWdof7mYcRWJ8NcUCQ4cyL33W7YRInxbfgSfeUiNLJcf7a_vPghAd0z6Yy6TS5gotgWZBCQ.webp',
        },
        {
          category: 'cosmopolitan',
          name: '코스모폴리탄',
          pirce: 1100,
          image:
            'https://i.namu.wiki/i/S_Z7yzhB493bIHsxLVPu8InEGo0V8-ZUwcs8aNOzQSOkGAFOqItUa7qpVjqKVrM1u9W3aX9lSMxfPlcK2pOaySwymNrzx1am7lFqzy5czBRx4lj9OwxlnjIGsNgUtkwpglMV8xi0tUIzPedUEnPy7g.webp',
        },
        {
          category: 'buleLagoon',
          name: '블루 라군',
          pirce: 1000,
          image:
            'https://i.namu.wiki/i/B97SSJpathUqXMWhhoMC2nxY7uK2Zi52D_DFOYzMSCTeHPwKbH2xYZCp0BRGZTDHkpUXX2bysuvVhMF3bNen8NDdk2KUJb2yHYv7KFQyKYsNsoCloFq70nDyipMGkuhk4l1vy0HSNtx0V113E7Qp3Q.webp',
        },
        {
          category: 'seaBreeze',
          name: '시 브리즈',
          pirce: 1100,
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
          pirce: 1100,
          image:
            'https://i.namu.wiki/i/XN5dMAiR9sao94N-gP7m5k-GwoUDMLoEHDjYqbPhCAGoYPVxQTQTaE29JU6yc8xpkuPvviGaNlPI2tmvDsP0HeBR7VELJx-1ATxcHVfx6t3Svd0sUBwq3fFmwiRXfWY_8PfKLdNj2AYiYvcYdj_hyQ.webp',
        },
        {
          category: 'bacardi',
          name: '바카디',
          pirce: 1000,
          image:
            'https://i.namu.wiki/i/F_B8mYSEcfMdLUrkJ8hEmfX8MyGaNE4hps4HCs1NTT4PHD7LecOfID3FjIKwN5piHBukA-7rxmSqj9-NrkUvYW9UdptHh-8Ye0vJvpyqP6gGqNpXSkcWXazwLJ55yC5gt-NDEsR3Kr9NaCP2Gl4F2A.webp',
        },
        {
          category: 'cubalibre',
          name: '쿠바 리브레',
          pirce: 900,
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
          pirce: 1100,
          image:
            'https://i.namu.wiki/i/-9I-WV1sV6EghdDrLY-iXw2lcn383YozTd45QxEdvuyciM8bThZ9jBaycZhsjCDUWoBkSTL67a0HcSOWypc5yXuZEdwVw1kkjIms-UVQLy_J9QyWC_PVC9D2nznpqeu96nfrFoKLEWxit6tPSiS_cA.webp',
        },
        {
          category: 'juneBug',
          name: '준벅',
          pirce: 900,
          image:
            'https://i.namu.wiki/i/GqRieyymqhooW8HejohkwzfMfYw_JmUHsb0yMgUajG0ZMH70_w8QQdTko8i5AQX6j8LMGcp9_7M0VGY21glD1F_m882UNC2yE_a3vuNg5rj8FwtuDZTrG8Wyxeo9YGDxoqgZSrJ2qG-NaG4uFt3M3g.webp',
        },
        {
          category: 'greenWidow',
          name: '그린 위도우',
          pirce: 1000,
          image:
            'https://i.namu.wiki/i/4OHkeJhkHe93JM_HRiPTVk6D7UHHkEyfjxzr45kAmfy1hoMN_I0L_x7SPZ79wIaKBegz9QjOt1Xhu2cYGFpJNg1lwTbeoHXTPspIINHks0pSu6HfRHesJRq4r413KlFPj8US-yRALAb18QajnG2F1A.webp',
        },
        {
          category: 'fuzzyNavel',
          name: '퍼지 네이블',
          pirce: 1000,
          image:
            'https://i.namu.wiki/i/AA90ehCDjZVftIDN9cbCRG17KG3IRfa641d9eR32yXXF6CQIdDvpqjZDLizD3vQvwTN3QbF1nATvXJfmBC9_NU4Ij_Avh9pi0zVj9p9yDjl6BDP0Qz-09R-ZyEoln2f__S_LmA7SG2uOhNaZiGkmkQ.webp',
        },
        {
          category: 'spritz',
          name: '스프릿츠',
          pirce: 1100,
          image:
            'https://i.namu.wiki/i/qkheVC0PlqJrwDkDosMnAIriUO1EmIzg9kl8Qf8B-xhC8M-PWNXLA9ooW43xVo76s_4O07u56X6tu8e3HBd6j33cdN8UgSbYB7CNMOnqQpnttil15LDwOrQ_X-S9zj0XTqeFweTnRoOLrrc6K7kvpw.webp',
        },
        {
          category: 'brainHemorrhage',
          name: '브레인 헤머리지',
          pirce: 1200,
          image:
            'https://i.namu.wiki/i/4iHGknkD05-C2dacdGYR71fxYhXHOSj-EkCfmk2ltS4e6_8AxFf7H6VNxz3ljUcKcaG7JlBHlAvJQSKMdaDCRaK91gaERgJpXC21nSoStwtdX0wOTes2ck5POvVeb7tMnMupWInrVltM2A_BF932ww.webp',
        },
      ],
    },
  ]

  return { menuList }
})

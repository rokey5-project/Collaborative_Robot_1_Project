<template>
  <div class="order-chart-container">
    <PieChart/>
    <BarChartTodaySales/>
    <BarChartSales/>
    <LineChartSales/>
    <LineChartPrice/>
  </div>
</template>

<script setup lang="ts">
import PieChart from '@/components/PieChart.vue';
import BarChartSales from '@/components/BarChartSales.vue';
import LineChartSales from '@/components/LineChartSales.vue';
import LineChartPrice from '@/components/LineChartPrice.vue';
import BarChartTodaySales from '@/components/BarChartTodaySales.vue';

</script>

<style scoped lang="scss">
.order-chart-container {
  width: 100%;
  padding: 30px;
  flex: 1;
  display: grid;
  grid-template-columns: 1fr 1fr 1fr;
  grid-template-rows: 1fr 1fr;
  gap: 10px;
}
</style>
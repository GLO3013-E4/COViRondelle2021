<template>
  <v-card class="d-flex justify-center" color="#ededed" height="280">
    <h3>{{ $t('consumptions.consumptionInformation') }}</h3>
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from 'vue-property-decorator';

@Component({
  components: {},
})
export default class ConsumptionInformation extends Vue {}
</script>

<style></style>

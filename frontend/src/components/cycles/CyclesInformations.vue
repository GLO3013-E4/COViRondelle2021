<template>
  <v-card class="d-flex justify-center" color="#ededed" height="300">
    <h3>Cycles informations</h3>
    <CycleList />
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from "vue-property-decorator";
import CycleList from "./CycleList.vue";

@Component({
  components: { CycleList: CycleList }
})
export default class CyclesInformations extends Vue {}
</script>

<style></style>

<template>
  <v-card class="d-flex justify-center" color="#ededed" height="300">
<<<<<<< HEAD:frontend/src/components/cycles/CyclesInformations.vue
    <h3>Cycles informations</h3>
    <CycleList />
=======
    <h3>Cycle information</h3>
>>>>>>> develop:frontend/src/components/cycles/CycleInformation.vue
  </v-card>
</template>

<script lang="ts">
import { Component, Vue } from "vue-property-decorator";
import CycleList from "../cycles/CycleList.vue";
@Component({
  components: { CycleList: CycleList }
})
export default class CycleInformation extends Vue {}
</script>

<style></style>

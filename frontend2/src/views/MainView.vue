<template>
  <v-container>
    <div class="d-flex justify-center mt-5">
      <h1 class="font-weight-medium">Robot culinaire</h1>
    </div>
    <div class="d-flex justify-center mt-5">
      <RobotInformations />
    </div>
  </v-container>
</template>

<script lang="ts">
import { Component, Vue } from "vue-property-decorator";
import RobotInformations from "../layouts/RobotInformations.vue";
@Component({
  components: { RobotInformations },
})
export default class MainView extends Vue {}
</script>

<style></style>

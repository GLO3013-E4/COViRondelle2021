<template>
  <div>hey</div>
</template>

<script lang="ts">
import { RobotConsumption } from '@/types/robotConsumption';
import { Component, Vue } from 'vue-property-decorator';
import { mapState } from 'vuex';

@Component({
  computed: {
    ...mapState(['robotConsumption']),
  },
})
export default class RemainingTime extends Vue {
  public robotConsumption!: RobotConsumption;
}
</script>
